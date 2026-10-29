#pragma once

#include <cstdint>
#include "obj_dir/Vaxi4_lite_acqu.h"
#include <iostream>

constexpr uint32_t REG_CTL = 0x00;
constexpr uint32_t REG_STATUS = 0x04;
constexpr uint32_t REG_PHASE_STEP = 0x08;
constexpr uint32_t REG_DOPPLER_STEP = 0x0C;

class AXI4 { 
private: 
    float axi_read(Vaxi4_lite_acqu* top, uint32_t araddr) { 
         top->s_axi_araddr = araddr;

    }
    float axi_write(Vaxi4_lite_acqu* top, uint32_t araddr, uint32_t data) { 
        
    }
};

int main(int argc, char* argv[]) { 
    return 0;
}
