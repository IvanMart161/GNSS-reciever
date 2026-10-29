#pragma once

#include <cstdint>
#include <iostream>

constexpr uint32_t REG_CTL          = 0x00;
constexpr uint32_t REG_STATUS       = 0x04;
constexpr uint32_t REG_PHASE_STEP   = 0x08;
constexpr uint32_t REG_DOPPLER_STEP = 0x0C;

template <class top>
class Axi4Driver {
private:
    top& top_;
    const int TIME_OUT = 100;
public: 
    Axi4Driver(top& t): top_(t) {}
    void write(uint32_t address, uint32_t data) { 
        top_.s_axi_awaddr = address;
        top_.s_axi_wdata = data;
        top_.s_axi_awvalid = 1;
        top_.s_axi_wvalid = 1;
        top_.s_axi_wstrb = 0x0F;
        bool w_sent = false;
        bool aw_sent = false;
    
        for(int i = 0; i < TIME_OUT && !(w_sent && aw_sent); ++i) { 
            top_.aclk = 0;
            top_.eval();
            top_.s_axi_awvalid = !aw_sent;
            top_.s_axi_wvalid = !w_sent;
            top_.aclk = 1;
            top_.eval();
            bool address_handshake = !aw_sent && top_.s_axi_awready;
            bool data_handshake = !w_sent && top_.s_axi_wready;
            aw_sent = aw_sent | address_handshake;
            w_sent = w_sent | data_handshake;
        }

        top_.s_axi_awaddr = 0;
        top_.s_axi_awready = 0;

        if(!(w_sent && aw_sent)) { 
            throw std::runtime_error("AXI write timed out");
        }
    }
    void read(uint32_t address) {
        
    }

};

int main(int argc, char* argv[]) { 
    return 0;
}
