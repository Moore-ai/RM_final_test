#include "conduction.hpp"
#include <iostream>

int main() {
    int n;
#ifdef IGNORE_INPUT
    n=4;
#else
    std::cout<<"视频序号: ";
    std::cin>>n;
#endif

    Conduction conduction(n);
    // conduction.work(25);
    // conduction.test_img("/home/lenovo/Pictures/截图/截图 2025-02-17 18-45-27.png");
    conduction.work();
    return 0;
}