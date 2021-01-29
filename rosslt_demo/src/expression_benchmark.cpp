#include "rosslt/tracked.h"
#include <cstdlib>
#include <ctime>
#include <string>
#include <iostream>
#include <chrono>
#include <vector>

int main(int argc, char* argv) {
    srand((unsigned) time(nullptr));

    std::chrono::duration<double> tracked_duration = std::chrono::duration<double>::zero();
    std::chrono::duration<double> apply_duration = std::chrono::duration<double>::zero();
    std::chrono::duration<double> reverse_duration = std::chrono::duration<double>::zero();
    std::chrono::duration<double> reverse_apply_duration = std::chrono::duration<double>::zero();

    int N = 10000;
    int M = 50;

    for(int n = 0; n < N; ++n) {
        Location loc("test", 0);
        Tracked<double> x (10.0, loc);
        std::vector<int> rand_numbers;
        for (int i = 0; i < M; ++i) {
            rand_numbers.push_back(rand()%4);
            rand_numbers.push_back(rand()%10+1);
        }

        auto start_tracked = std::chrono::steady_clock::now();
        for (int i = 0; i < M; ++i) {
            switch(rand_numbers[2*i]) {
            case 0:
                x = x + (rand_numbers[2*i+1]);
                break;
            case 1:
                x = x - (rand_numbers[2*i+1]);
                break;
            case 2:
                x = x * (rand_numbers[2*i+1]);
                break;
            case 3:
                x = x / (rand_numbers[2*i+1]);
                break;
            }
        }
        tracked_duration += std::chrono::steady_clock::now() - start_tracked;

        std::string exp = x.get_location().at(".").expression;

        auto start_apply = std::chrono::steady_clock::now();
        double a = applyExpression(10.0, exp);
        apply_duration += std::chrono::steady_clock::now() - start_apply;

        auto start_reverse = std::chrono::steady_clock::now();
        std::string rev = reverseExpression(exp);
        reverse_duration += std::chrono::steady_clock::now() - start_reverse;

        auto start_reverse_apply = std::chrono::steady_clock::now();
        double b = applyExpression(a, rev);
        reverse_apply_duration += std::chrono::steady_clock::now() - start_reverse_apply;

        if (b < 9.9 || b > 10.1)
            std::cout << "Error: " << b << " != 10 (" << exp << ", rev: " << rev << ")" << std::endl;

    }

    std::cout << "tracked: " << tracked_duration.count() * 1000000 / N << "µs" << std::endl;
    std::cout << "apply: " << apply_duration.count() * 1000000 / N << "µs" << std::endl;
    std::cout << "reverse: " << reverse_duration.count() * 1000000 / N << "µs" << std::endl;
    std::cout << "reverse_apply: " << reverse_apply_duration.count() * 1000000 / N << "µs" << std::endl;
}
