// disturbance_estimator.cpp
// Estimador de perturbaciones que publica los datos para logging

#include <px4_platform_common/module.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/disturbance_estimator.h>

#include <matrix/matrix/Vector.hpp>
#include <matrix/matrix/SquareMatrix.hpp>
#include <matrix/matrix/Dcm.hpp>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <errno.h>

class DisturbanceEstimator : public ModuleBase<DisturbanceEstimator> {
public:

    static int task_spawn(int argc, char *argv[]) {
        _task_id = px4_task_spawn_cmd("disturbance_estimator",
                                      SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
                                      1500,
                                      reinterpret_cast<px4_main_t>(&DisturbanceEstimator::run_trampoline),
                                      argv);
        return _task_id < 0 ? -errno : 0;
    }

    static int custom_command(int argc, char *argv[]) {
        PX4_WARN("Unknown command");
        return print_usage();
    }

    static int print_usage() {
        PX4_INFO("Usage: disturbance_estimator {start}");
        return 0;
    }

    static int run_trampoline(int argc, char *argv[]) {
        DisturbanceEstimator instance;
        instance.run();
        return 0;
    }

    void run() {
        PX4_INFO("Disturbance Estimator running");

        int accel_sub = orb_subscribe(ORB_ID(vehicle_acceleration));
        int attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int thrust_sp_sub = orb_subscribe(ORB_ID(vehicle_thrust_setpoint));
        int gyro_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
        int status_sub = orb_subscribe(ORB_ID(vehicle_status));

        orb_advert_t disturbance_pub = nullptr;

        const float drone_mass = 1.5f;
        const float gravity = 9.81f;
        const float dt = 0.02f;
        const float K_eta = 0.1f;

        matrix::SquareMatrix<float, 3> I;
        I(0, 0) = 0.02f;
        I(1, 1) = 0.02f;
        I(2, 2) = 0.04f;

        matrix::Vector3f m_hat{0.f, 0.f, 0.f};
        matrix::Vector3f integral_term{0.f, 0.f, 0.f};

        uint64_t last_print_time = 0;
        bool waiting_message_shown = false;

        while (!should_exit()) {
            px4_usleep(2000000); // 50 Hz

            vehicle_status_s status{};
            orb_copy(ORB_ID(vehicle_status), status_sub, &status);
            bool armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

            if (!armed) {
                if (!waiting_message_shown) {
                    PX4_INFO("Esperando a que el dron esté armado...");
                    waiting_message_shown = true;
                }
                continue;
            } else {
                waiting_message_shown = false;
            }

            bool updated = false;

            orb_check(accel_sub, &updated);
            if (!updated) continue;
            vehicle_acceleration_s accel{};
            orb_copy(ORB_ID(vehicle_acceleration), accel_sub, &accel);
            matrix::Vector3f accel_body(accel.xyz);

            orb_check(attitude_sp_sub, &updated);
            if (!updated) continue;
            vehicle_attitude_s att_sp{};
            orb_copy(ORB_ID(vehicle_attitude), attitude_sp_sub, &att_sp);
            matrix::Dcmf R_bi(att_sp.q);

            orb_check(thrust_sp_sub, &updated);
            if (!updated) continue;
            vehicle_thrust_setpoint_s thrust_sp{};
            orb_copy(ORB_ID(vehicle_thrust_setpoint), thrust_sp_sub, &thrust_sp);
            matrix::Vector3f thrust_body(thrust_sp.xyz);

            // Sanitizar thrust_body
            for (int i = 0; i < 3; ++i) {
                if (!PX4_ISFINITE(thrust_body(i)) || fabsf(thrust_body(i)) > 50.0f) {
                    thrust_body(i) = 0.0f;
                }
            }
            if (thrust_body(2) > 0.0f) {
                thrust_body(2) = -fabsf(thrust_body(2));
            }

            // DEBUG: Imprimir thrust_body
            PX4_INFO("thrust_body: [x=%.3f, y=%.3f, z=%.3f]",
                (double)thrust_body(0), (double)thrust_body(1), (double)thrust_body(2));

            orb_check(gyro_sub, &updated);
            if (!updated) continue;
            vehicle_angular_velocity_s gyro{};
            orb_copy(ORB_ID(vehicle_angular_velocity), gyro_sub, &gyro);
            matrix::Vector3f omega(gyro.xyz);

            matrix::Vector3f accel_inertial = R_bi.transpose() * accel_body;
            matrix::Vector3f thrust_inertial = R_bi.transpose() * (thrust_body * drone_mass * gravity);
            matrix::Vector3f m = drone_mass * accel_inertial - thrust_inertial;

            //Clamp m para evitar desbordes
            for (int i = 0; i < 3; ++i) {
                if (!PX4_ISFINITE(m(i)) || fabsf(m(i)) > 1000.0f) {
                    m(i) = 0.0f;
                }
            }

            matrix::Vector3f Iomega = I * omega;
            matrix::Vector3f Iomega_cross_omega = Iomega % omega;

            integral_term += (m + Iomega_cross_omega - m_hat) * dt;
            m_hat = K_eta * (Iomega - integral_term);

            //Clamp m_hat para evitar desbordes
            for (int i = 0; i < 3; ++i) {
                if (!PX4_ISFINITE(m_hat(i)) || fabsf(m_hat(i)) > 1000.0f) {
                    m_hat(i) = 0.0f;
                }
            }

            uint64_t now = hrt_absolute_time();

            if (now - last_print_time >= 2000000) {
                PX4_INFO("Disturbance force: [x=%.3f, y=%.3f, z=%.3f] N",
                         (double)m(0), (double)m(1), (double)m(2));
                PX4_INFO("Estimated moment (m_hat): [x=%.3f, y=%.3f, z=%.3f]",
                         (double)m_hat(0), (double)m_hat(1), (double)m_hat(2));
                last_print_time = now;
            }

            disturbance_estimator_s dist_msg{};
            dist_msg.timestamp = now;
            dist_msg.force_x = m(0);
            dist_msg.force_y = m(1);
            dist_msg.force_z = m(2);
            dist_msg.moment_x = m_hat(0);
            dist_msg.moment_y = m_hat(1);
            dist_msg.moment_z = m_hat(2);

            if (disturbance_pub == nullptr) {
                disturbance_pub = orb_advertise(ORB_ID(disturbance_estimator), &dist_msg);
            } else {
                orb_publish(ORB_ID(disturbance_estimator), disturbance_pub, &dist_msg);
            }
        }
    }

private:
    static int _task_id;
};

int DisturbanceEstimator::_task_id = -1;

extern "C" __EXPORT int disturbance_estimator_main(int argc, char *argv[]) {
    return DisturbanceEstimator::main(argc, argv);
}
