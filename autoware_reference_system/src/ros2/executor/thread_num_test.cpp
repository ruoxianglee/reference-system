#include <thread>
#include <iostream>

int main(int argc, char * argv[])
{
    for (int i = 0; i++; i < 1000)
    {
        unsigned int n = std::thread::hardware_concurrency();
        std::cout << n << " concurrent threads are supported.\n";
    }

    return 0;
}
