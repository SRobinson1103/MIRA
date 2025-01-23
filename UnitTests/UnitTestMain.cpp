#include "MIRAMathTests.h"

int main()
{
    auto& tests = TestRegistry::instance().tests;
    int passed = 0, failed = 0;

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "PASS: " << name << "\n";
            passed++;
        }
        catch (...)
        {
            std::cerr << "FAIL: " << name << "\n";
            failed++;
        }
    }

    std::cout << "\nResults: " << passed << " passed, " << failed << " failed.\n";
    system("pause");
    return failed > 0 ? 1 : 0;
}