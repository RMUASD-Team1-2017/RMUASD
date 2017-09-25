/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <stefan@stefanrvo.dk> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */

#pragma once
#include "libmsr145.hpp"
#include <string>
#include <ostream>
typedef std::pair<float, std::vector<active_measurement::active_measurement> > measure_interval_pair;


class MSRTool : public MSRDevice
{
    private:
        bool light_sensor = false;
    public:
        MSRTool(std::string _portname) : MSR_Base(_portname), MSRDevice(_portname)
            {}
        virtual ~MSRTool()
            {}
        virtual void print_status();
        virtual void set_lightsensor() { light_sensor = true; }
        using MSR_Writer::start_recording;
        virtual void start_recording(std::string starttime_str, std::string stoptime_str, bool ringbuff);
        virtual void start_recording(startcondition start_option, bool ringbuff); //only to use for push options
        virtual std::string get_time_str(struct tm(MSRTool::*get_func)(void));
        virtual std::string get_device_time_str();
        virtual std::string get_start_time_str();
        virtual std::string get_end_time_str();
        virtual std::string get_interval_string();
        virtual float convert_to_unit(sampletype type, int16_t value, float conversion_factor = 0);
        virtual std::string get_sensor_str(sampletype type, int16_t value);
        virtual std::string get_start_settings_str();
        virtual std::string get_sample_limit_str(sampletype type);
        virtual std::string get_limits_str();
        virtual std::string get_calibration_str();
        std::string get_calibration_type_str(active_calibrations::active_calibrations type);
        virtual void get_type_str(sampletype type, std::string &type_str, std::string &unit_str);
        virtual void list_recordings();
        virtual void extract_record(uint32_t rec_num, std::string seperator, std::ostream &out_stream);
        virtual std::string create_csv(std::vector<sample> &samples, std::string &seperator);
        virtual void set_measurement_and_timers(std::vector<measure_interval_pair> interval_typelist);
        virtual void set_name(std::string name);
        virtual void set_calibration_date(uint16_t year, uint16_t month, uint16_t day);
        virtual void set_calib_name(std::string name);
        virtual int16_t convert_from_unit(sampletype type, int16_t value, float conversion_factor = 0);
        virtual void set_calibrationpoints(active_calibrations::active_calibrations type, std::vector<float> points);
        virtual std::string get_firmware_version_str();
        using MSRDevice::set_time;
        virtual void set_time(std::string timestr);
        virtual void set_limit(sampletype type, float limit1, float limit2, limit_setting record_limit, limit_setting alarm_limit);
        virtual void print_sensors(std::vector<sampletype> sensor_to_poll);
        using MSRDevice::set_limit;
    private:
};
