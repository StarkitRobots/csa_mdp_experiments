#pragma once

#include <random>
#include <Eigen/Dense>
#include "odometry/odometry_displacement_model.h"
#include "odometry/odometry_noise_model.h"

namespace csa_mdp {

/**
 * Odometry
 *
 * Utilities for differentiate and
 * integrate the robot displacement.
 * Manage displacement and noise model.
 */
class Odometry
{
    public:

        Odometry();
        /**
         * Initialization with displacement
         * and noise model types
         */
        Odometry(
            OdometryDisplacementModel::Type typeDisplacement,
            OdometryNoiseModel::Type typeNoise 
                = OdometryNoiseModel::NoiseDisable);

        /**
         * Return displacement and noise 
         * model internal types.
         */
        OdometryDisplacementModel::Type getDisplacementType() const;
        OdometryNoiseModel::Type getNoiseType() const;
        
        /**
         * Return current or default
         * parameters for displacement 
         * and noise models.
         */
        Eigen::VectorXd getParameters() const;

        /**
         * Assign given displacement and noise 
         * model parameters. 
         * If given parameters does not
         * comply with min or max bounds, 
         * a positive distance (for fitness scoring)
         * from given parameters is given.
         * Else, the parameters are assigned
         * and zero is returned.
         */
        double setParameters(
            const Eigen::VectorXd& params);

        std::vector<std::string> getParametersNames() const;

        /**
         * Return parameter normalization 
         * coefficients for displacement 
         * and noise models.
         */
        Eigen::VectorXd getNormalization() const;

        /**
         * Print current parameters on
         * standart output
         */
        void printParameters() const;

        /**
         * Reset to zero or given pose integrated state
         * and mark internal data to be re initialized
         */
        void reset();
        void reset(const Eigen::Vector3d& pose);

        /**
         * Update pose state by integrating given
         * relative displacement between two
         * support foot transition (dX, dY, dTheta) 
         * (meter, radian).
         */
        void updateFullStep(
            const Eigen::Vector3d& deltaPose,
            std::default_random_engine* engine);

        /**
         * Sample the difference of position in the robot referential
         */
        Eigen::Vector3d getDiffFullStep(
            const Eigen::Vector3d& deltaPose,
            std::default_random_engine* engine) const;

        /**
         * Return current corrected odometry state
         * [x,y,theta]
         */
        const Eigen::Vector3d& state() const;
        
        /**
         * Compute odometry displacement
         * vector from state1 to state2
         */
        Eigen::Vector3d odometryDiff(
            const Eigen::Vector3d& state1, 
            const Eigen::Vector3d& state2) const;

        /**
         * Integrate given odometry diff vector
         * to given state and update it
         */
        void odometryInt(
            const Eigen::Vector3d& diff,
            Eigen::Vector3d& state) const;

        /**
         * Build an odometry from the specified file
         */
        void saveToFile(const std::string & path) const;

        /**
         * Build an odometry from the specified file
         */
        void loadFromFile(const std::string & path);

    private:

        /**
         * Displacement and noise models
         */
        OdometryDisplacementModel _modelDisplacement;
        OdometryNoiseModel _modelNoise;

        /**
         * If false, the next update() will
         * initialize internal data to match
         * current input Model state
         */
        bool _isInitialized;

        /**
         * Input Model robot pose (self in <
         * origin) in world frame at last 
         * support foot swap
         */
        Eigen::Vector3d _last;

        /**
         * Output corrected robot pose in
         * world frame at last support foot swap.
         */
        Eigen::Vector3d _state;

        /**
         * Output corrected robot pose 
         * in world frame integrated at each
         * update
         */
        Eigen::Vector3d _corrected;

        /**
         * Robot self displacement at last support
         * foot swap used to compute models with
         * history information
         */
        Eigen::Vector3d _lastDiff;
};

}
