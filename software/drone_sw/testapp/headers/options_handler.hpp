#include "msr145_tool.hpp"
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include "msr145_tool.hpp"

#define COMMAND_LINE_ERROR 1
#define UNHANDLED_EXCEPTION 2


namespace po = boost::program_options;
class options_handler
{
    private:
        void handlelimits(po::variables_map &vm, MSRTool &msr);
        void handle_sampling_args(po::variables_map &vm, MSRTool &msr);
        int handle_extract_args(__attribute__((unused))po::variables_map &vm, MSRTool &msr);
        int handle_start_args(po::variables_map &vm, MSRTool &msr);
        int add_to_intervallist(std::vector<float> &interval_list, active_measurement::active_measurement type,
            std::vector<measure_interval_pair> &interval_type_list);
        limit_setting parse_alarm_limit(std::string limit_str);
        void handle_get_sensors(po::variables_map &vm, MSRTool &msr);
        limit_setting parse_recording_limit(std::string limit_str);
        bool measure_interval_pair_cmp(measure_interval_pair &p1, measure_interval_pair &p2);
        int handle_command(po::variables_map &vm, MSRTool *&msr);
        po::positional_options_description p;
        bool device_required;
    public:
        po::options_description *desc;
        int handle_args(int argc, char const **argv, MSRTool *&msr);

        int handle_tokens(std::vector<std::string> &tokens, MSRTool *&msr);

        options_handler(bool _device_required = false);
        ~options_handler();

};
