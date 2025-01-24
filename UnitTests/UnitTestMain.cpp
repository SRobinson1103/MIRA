#include "MIRAMathTests.h"
#include "MIRACollisionTests.h"

int main()
{
    auto& tests = TestRegistry::instance().tests;
    int passed = 0, failed = 0;

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << COLOR_BLUE <<  "PASS: " << COLOR_RESET << name << "\n";
            passed++;
        }
        catch (...)
        {
            std::cerr << COLOR_RED <<  "FAIL: " << COLOR_RESET << name << "\n";
            failed++;
        }
    }

    std::cout << "\nResults: " << passed << COLOR_GREEN << " PASSED, " << COLOR_RESET << failed << COLOR_YELLOW << " FAILED.\n" << COLOR_RESET;

    system("pause");
    return failed > 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}