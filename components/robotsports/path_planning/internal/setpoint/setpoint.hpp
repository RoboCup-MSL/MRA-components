#ifndef SETPOINT_HPP
#define SETPOINT_HPP 1


typedef struct Segment {
    std::vector<double> p;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> t;
    std::vector<double> dt;
} Segment_t;


// std::tuple<std::vector<Segment_t>, std::vector<double>, std::vector<double>, std::vector<double>> balance_xy(
//     std::vector<Segment_t> segment, 
//     const std::vector<double>& p0, 
//     const std::vector<double>& v0, 
//     const std::vector<double>& pe, 
//     const std::vector<double>& ve, 
//     const std::vector<double>& vm, 
//     const std::vector<double>& am, 
//     const std::vector<double>& dm);


double wrap(double angle, double reference);

Segment_t move_at_constant_vel(const Segment_t& segment_in,
                                const std::vector<double>& p0, 
                                const std::vector<double>& v0, 
                                const std::vector<double>& t0, 
                                const std::vector<double>& dt);

Segment_t move_to_vel(const Segment_t& segment_in, 
                      const std::vector<double>& p0, const std::vector<double>& v0, 
                      const std::vector<double>& t0, const std::vector<double>& ve, 
                      const std::vector<double>& am, const std::vector<double>& dm);

std::pair<std::vector<double>, std::vector<double>> get_max_velocity(const Segment_t& segment_in, const std::vector<double>& p0, 
                                                                     const std::vector<double>& v0, const std::vector<double>& pe, 
                                                                     const std::vector<double>& ve, const std::vector<double>& vm, 
                                                                     const std::vector<double>& am, const std::vector<double>& dm);


void get_segments(std::vector<Segment_t>& segment, const std::vector<double>& p0, 
                    const std::vector<double>& v0, const std::vector<double>& pe, 
                    const std::vector<double>& ve, const std::vector<double>& vm, 
                    const std::vector<double>& am, const std::vector<double>& dm);





typedef struct Traject {
    std::vector<double> t;
    std::vector<std::vector<double>> p;
    std::vector<std::vector<double>> v;
    std::vector<std::vector<double>> a;
    std::vector<std::vector<int>> segment_id;
} Traject_t;

std::tuple<std::vector<double>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<bool>>> traj_segment(
    const std::vector<std::map<std::string, std::vector<double>>>& segment, const std::vector<double>& time);

// std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<bool>>>
//         traj_segment(const std::vector<std::map<std::string, std::vector<double>>>& segment, const std::vector<double>& time);


std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> combine_segment_data(
            const std::vector<std::map<std::string, std::vector<double>>>& segment);

// std::tuple<std::vector<double>, std::vector<std::vector<double>>, std::vector<vector<double>>, std::vector<std::vector<double>>>
//         combine_segment_data(const std::vector<std::map<std::string, std::vector<double>>>& segment);

Traject_t traj1(Traject traject, const std::vector<std::map<std::string, std::vector<double>>>& segment, double Ts);



struct Input {
    struct Robot {
        int skillID;
    } robot;
};

struct Data {
    Traject_t traj;
    Input input;
    struct Parameters {
        double Ts_predict;
    } par;
};

std::vector<double> arange(int start, int end);

Data traj_predict(Data d, const std::vector<std::map<std::string, std::vector<double>>>& segment);




#endif