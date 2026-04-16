#include "Vsmolrv64_tb.h"
#include "verilated.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vsmolrv64_tb* top = new Vsmolrv64_tb;
    while (!Verilated::gotFinish()) {
        top->clock = 0; top->eval();
        top->clock = 1; top->eval();
    }
    delete top;
    return 0;
}
