/**
 * Copyright (C) 2019-2021 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "xcl2.hpp"
#include "cmdlineparser.h"
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <numeric>
#include <future>

// XRT includes
#include "experimental/xrt_bo.h"
#include "experimental/xrt_device.h"
#include "experimental/xrt_kernel.h"

std::vector<double> run_write_bandwidth(std::string binaryFile, int device_index, int repeat)
{
    int iter = 1;
    std::vector<double> throughputs_GBps;
    throughputs_GBps.reserve(repeat);

    std::cout << "Open the device " << device_index << std::endl;
    auto device = xrt::device(device_index);
    std::cout << "Load the xclbin " << binaryFile << std::endl;
    auto uuid = device.load_xclbin(binaryFile);

    auto krnl_write = xrt::kernel(device, uuid, "write_bandwidth");

    size_t bufsize = 512 * 1024 * 1024;

    xrt::bo::flags flags = xrt::bo::flags::host_only;
    auto hostonly_bo_out = xrt::bo(device, bufsize, flags, krnl_write.group_id(0));

    double dbytes = bufsize;

    // Map the contents of the buffer object into host memory
    auto hostonly_bo_out_map = hostonly_bo_out.map<char *>();

    std::fill(hostonly_bo_out_map, hostonly_bo_out_map + bufsize, 0);

    auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < repeat; i++)
    {
        if (xcl::is_emulation())
        {
            iter = 2;
            if (bufsize > 8 * 1024)
                break;
        }

        auto run_write = krnl_write(hostonly_bo_out, bufsize, iter);
        run_write.wait();

        auto end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        start = end;
        double msduration = duration / iter;

        /* Profiling information */
        double dsduration = msduration / ((double)1000000);
        double bpersec = (dbytes / dsduration);
        double gbpersec = (bpersec) / ((double)1024 * 1024 * 1024);

        throughputs_GBps.push_back(gbpersec);

        // std::cout << "Write Throughput = " << gbpersec << " (GB/sec) for device " << device_index << "\n";
    }

    return throughputs_GBps;
}

std::vector<double> run_sync(std::string binaryFile, int device_index, int repeat)
{
    std::vector<double> throughputs_GBps;
    throughputs_GBps.reserve(repeat);

    std::cout << "Open the device " << device_index << std::endl;
    auto device = xrt::device(device_index);
    std::cout << "Load the xclbin " << binaryFile << device_index << std::endl;
    auto uuid = device.load_xclbin(binaryFile);

    auto krnl_read = xrt::kernel(device, uuid, "read_bandwidth");

    size_t bufsize = 512 * 1024 * 1024;

    auto bo_out = xrt::bo(device, bufsize, xrt::bo::flags::normal, krnl_read.group_id(0));
    auto bo_out_map = bo_out.map<char *>();
    double dbytes = bufsize;

    std::fill(bo_out_map, bo_out_map + bufsize, 0);

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < repeat; i++)
    {
        if (xcl::is_emulation())
        {
            if (bufsize > 8 * 1024)
                break;
        }

        bo_out.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        auto end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        double msduration = duration;
        start = end;

        /* Profiling information */
        double dsduration = msduration / ((double)1000000);
        double bpersec = (dbytes / dsduration);
        double gbpersec = (bpersec) / ((double)1024 * 1024 * 1024);

        throughputs_GBps.push_back(gbpersec);

        // std::cout << "Write Throughput = " << gbpersec << " (GB/sec) for device " << device_index << "\n";
    }

    return throughputs_GBps;
}

int main(int argc, char *argv[])
{
    // Command Line Parser
    sda::utils::CmdLineParser parser;

    // Switches
    //**************//"<Full Arg>",  "<Short Arg>", "<Description>", "<Default>"
    parser.addSwitch("--xclbin_file", "-x", "input binary file string", "");
    parser.parse(argc, argv);

    // Read settings
    std::string binaryFile = parser.value("xclbin_file");

    if (argc < 2)
    {
        parser.printHelp();
        return EXIT_FAILURE;
    }

    std::vector<std::future<std::vector<double>>> run_write_bandwidth_futures;
    for (size_t i = 0; i < 8; i++)
    {
        std::future<std::vector<double>> run_write_bandwidth_future = std::async(std::launch::async, run_write_bandwidth, binaryFile, i, 400);
        run_write_bandwidth_futures.emplace_back(std::move(run_write_bandwidth_future));
    }

    int i = 0;
    for (auto &run_write_bandwidth_future : run_write_bandwidth_futures)
    {
        auto throughputs_GBps = run_write_bandwidth_future.get();
        std::ofstream output_file("./device" + std::to_string(i) + ".txt");

        std::ostream_iterator<double> output_iterator(output_file, "\n");
        std::copy(std::begin(throughputs_GBps), std::end(throughputs_GBps), output_iterator);
        i++;
    }

    return EXIT_SUCCESS;
}
