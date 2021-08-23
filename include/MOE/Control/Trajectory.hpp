#pragma once
#include <Moe/Control/WayPoint.hpp>
#include <Mahi/Util/Math/Constants.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <vector>

namespace moe {

    //==============================================================================
    // CLASS DECLARATION
    //==============================================================================

    class Trajectory {

    public:
        enum Interp {Linear};

    public:
        /// Returns vector of n time-linearly-interpolated points with equal spacing
        /// in time. Include initial/final set whether or not they will be included in
        /// the output, and count towards n.
        static std::vector<WayPoint> linspace_points(const WayPoint &initial,
            const WayPoint &final, std::size_t n,
            bool include_initial = true,
            bool include_final = true);

        /// Linearly interpolates between the two points based on the given time.
        /// Saturates to initial and final values if time is outside of range
        static WayPoint linear_time_interpolate(const WayPoint &initial, const WayPoint &final,
            const mahi::util::Time &t);

        /// Constructor
        Trajectory();
        Trajectory(std::size_t path_dim, const std::vector<WayPoint> &waypoints,
            Interp interp_method = Interp::Linear,
            const std::vector<double> &max_diff = {mahi::util::INF });

        /// Returns a position along the trajectory at the specific instant in time
        /// using one of the available interpolation methods
        std::vector<double>
            at_time(const mahi::util::Time &instant, Interp interp_method = Interp::Linear) const;

        /// Index-based read access to waypoints
        const WayPoint &operator[](std::size_t index) const;

        /// Index-based write access to waypoints
        bool add_waypoint(std::size_t index, const WayPoint &waypoint);

        /// Return the first waypoint
        const WayPoint &front() const;

        /// Return the last waypoint
        const WayPoint &back() const;

        /// Sets the waypoints that make the trajectory
        bool set_waypoints(std::size_t path_dim, const std::vector<WayPoint> &waypoints,
            Interp interp_method = Interp::Linear,
            const std::vector<double> &max_diff = { mahi::util::INF });

        /// Sets the method of interpolation to be used
        void set_interp_method(Interp interp_method);

        /// Sets the maximum allowable time derivative on the trajectory
        void set_max_diff(const std::vector<double> &max_diff);

        /// Returns whether or not the trajectory is empty, having no points
        bool empty() const;

        /// Return the length of the trajectory, meaning the number of Points that can be stored
        std::size_t size() const;

        /// Changes the length of the trajectory, meaning the number of Points that can be stored
        void resize(std::size_t new_size);

        /// Returns the path dimension
        std::size_t get_dim() const;

        /// Clears the waypoints, setting the trajectory to empty
        void clear();

        /// Adds a single waypoint to the end of the list
        bool push_back(const WayPoint &waypoint);

        /// Validae that the trajectory has correct dimensions, smoothness, and time properties
        bool validate() const;

        /// Overload the << stream operator with a Trajectory as the rhs argument
        friend std::ostream& operator<<(std::ostream& os, const Trajectory& trajectory);

    private:
        bool check_max_diff() const;
        
        bool check_waypoints() const;

        /// Checks if current trajectory satisfies smoothness criterion based on
        /// max_diff
        bool is_path_smooth() const;

    private:
        bool is_empty_; ///< whether or not this trajectory is empty, having no points

        std::size_t path_dim_; ///< dimensionality of the path

        Interp interp_method_; ///< method of interpolation to be used: None = no
                               ///< points added to the path, Linear = new points
                               ///< automatically added between waypoints using linear
                               ///< interpolation at the requested resolution
        
        std::vector<WayPoint> waypoints_; ///< vector of underlying waypoints

        std::vector<mahi::util::Time>
            times_; ///< vector of the times associated with the underlying waypoints

        std::vector<double> max_diff_;

    };

} // namespace moe