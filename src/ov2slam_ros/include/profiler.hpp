/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once

#include <string>

#include <sstream>
#include <chrono>
#include <map>
#include <vector>
#include <functional>
#include <cmath>
#include <mutex>
#include <fstream>
#include <cstdlib>

class Profiler {
public:
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<float, std::milli>;
    std::mutex mutex_;

    class State {
        size_t n_ = 0;
        float mu_ = 0.0f, s_ = 0.0f, min_= std::numeric_limits<float>::max(), max_ = -std::numeric_limits<float>::max();
        float acc_ = 0.0f;
      public:
        
        void update(float dt) {
            n_++;
            dt += acc_;
            float err =dt - mu_;
            mu_ += err / n_;
            
            s_ += (dt - mu_)*err;
            
            if (dt < min_)
              min_ = dt;
              
            if (dt > max_)
              max_ = dt;
            
            acc_ = 0.0f;
        }
        
        void store(float dt)
        {
          acc_ += dt;
        }
        
        bool have_store()
        {
          return acc_ != 0.0;
        }
        
        float store_value()
        {
          return acc_;
        }
        
        float var() const {
          return (n_<2) ? 0.0: s_/(n_-1);
        }
        
        float std() const {
          return (n_<2) ? 0.0: std::sqrt(s_/(n_-1));
        }
        
        float mean() const {
          return mu_;
        }
        
        float min() const {
          return min_;
        }
        
        float max() const {
          return max_;
        }

        std::string show() const {
            return std::to_string(mu_) + " \u00B1 "
                    + std::to_string( std() ) 
                    + " [" + std::to_string(min_) 
                    + "," + std::to_string(max_) 
                    + "] ms";
        }
    };

private:
    Profiler()= default;
    ~Profiler()= default;
    Profiler(const Profiler&)= delete;
    Profiler& operator=(const Profiler&)= delete;

public:
    static Profiler& getInstance(){
        static Profiler instance;
        // volatile int dummy{};
        return instance;
    }
    
    static void Start(const std::string & name) {
        Profiler::getInstance().start(name);
    }
    
    static Duration Stop(const std::string & name) {
        return Profiler::getInstance().stop(name);
    }
    
    static Duration Pause(const std::string & name) {
        return Profiler::getInstance().pause(name);
    }
    
    static void Restart(const std::string & name) {
        Profiler::getInstance().restart(name);
    }

    static void Display(const std::string & name) {
        std::vector<std::string> vname = {name};
        Profiler::getInstance().displayTimeLogs(vname);
    }
    
    static void StopAndDisplay(const bool display, const std::string & name) {
        Duration t = Stop(name);
        if( display )
          std::cout << "\n\n >>> " << name << " : " << t.count() << " ms \n\n";
    }

    void restart(const std::string & name) {
        start(name);
    }
    
    void start(const std::string & name) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        start_map_[name] = std::chrono::high_resolution_clock::now();
    }

    Duration pause(const std::string & name)
    {
       return stop(name, true);  
    };

    Duration stop(const std::string & name, bool pause = false) {
        Clock::time_point chrono_end = Clock::now();
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto stime = start_map_.find(name);
        if( stime == start_map_.end())
        {
            auto it =  timing_map_.find(name);
            if (it != timing_map_.end() && it->second.have_store())
            {
              float val = it->second.store_value();
              it->second.update(0.0);
              return Duration(val);
            }
            
            return Duration();
        }
        
        Clock::time_point chrono_begin = stime->second;
        start_map_.erase(stime);

        Duration duration = std::chrono::duration_cast<Duration>(chrono_end - chrono_begin);
        if (!pause) {
          timing_map_[name].update(duration.count());
        }
        else {
          timing_map_[name].store(duration.count());
        }

        return duration;
    }

    std::string displayTimeLogs(const std::vector<std::string> & name_list = std::vector<std::string>()) const
    {
        std::stringstream ss;
        ss << "\n\n****************************************************************";
        ss << "\n   Time Logs Summary (average time \u00B1 std [min , max]  [ms])\n";

        if(name_list.empty()) {
            for(const auto& el : timing_map_) {
                ss << std::endl << ">>> " << el.first << " : " << el.second.show();
            }
        } else {
            for(const std::string &name : name_list) {
                auto it = timing_map_.find(name);
                if( it != timing_map_.end()) {
                    auto & state = it->second;
                    ss << std::endl << ">>> " << name << " : " << state.show() ;
                } else {
                    ss << std::endl << ">>>" << "timer " << name
                       << "  not found";
                }
            }
        }

        ss << "\n***********************************\n";
        return ss.str();
    }

    // Export all timing statistics to CSV for paper benchmarks
    void exportCSV(const std::string & filepath) const {
        std::ofstream ofs(filepath);
        if (!ofs.is_open()) return;
        ofs << "timer,mean_ms,std_ms,min_ms,max_ms\n";
        for (const auto& el : timing_map_) {
            ofs << el.first << ","
                << el.second.mean() << ","
                << el.second.std() << ","
                << el.second.min() << ","
                << el.second.max() << "\n";
        }
    }

    static void ExportCSV(const std::string & filepath) {
        Profiler::getInstance().exportCSV(filepath);
    }

    // ── Per-frame event CSV sink (benchmark, HIL) ─────────────────────────
    // Mirrors ORB-SLAM2's ORB_BENCH_TIMING_CSV / ScopedBenchmarkTimer: if the
    // env var OV2_BENCH_TIMING_CSV is set, each call appends ONE row and flushes
    // it immediately, so per-frame timing survives a `docker stop` mid-run (the
    // aggregate ExportCSV above only fires on a graceful writeResults(), which a
    // HIL stop does not reach). No-op when the env var is unset, so the call
    // site can stay compiled in unconditionally at ~zero cost.
    static void LogEvent(const std::string & category, float duration_ms, long frame_id) {
        Profiler::getInstance().logEvent(category, duration_ms, frame_id);
    }

    void logEvent(const std::string & category, float duration_ms, long frame_id) {
        std::lock_guard<std::mutex> lock(event_mutex_);
        if( !event_csv_checked_ ) {
            event_csv_checked_ = true;
            const char* path = std::getenv("OV2_BENCH_TIMING_CSV");
            if( path && path[0] != '\0' ) {
                event_csv_.open(path, std::ios::out | std::ios::trunc);
                if( event_csv_.is_open() )
                    event_csv_ << "wall_ts,category,duration_ms,frame_id\n";
            }
        }
        if( !event_csv_.is_open() )
            return;
        double wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        event_csv_ << std::fixed << wall << "," << category << ","
                   << duration_ms << "," << frame_id << "\n";
        event_csv_.flush();
    }

private:
    std::map<std::string, Clock::time_point> start_map_;
    std::map<std::string, State> timing_map_;

    std::ofstream event_csv_;
    std::mutex event_mutex_;
    bool event_csv_checked_ = false;
};
