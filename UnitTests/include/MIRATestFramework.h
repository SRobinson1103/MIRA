#ifndef MIRA_TEST_FRAMEWORK_H
#define MIRA_TEST_FRAMEWORK_H

#include <vector>
#include <string>
#include <iostream>

#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_RESET "\033[0m"

// Test case function signature
using TestCase = void(*)();

// Test registry
struct TestRegistry
{
    std::vector<std::pair<std::string, TestCase>> tests;
    static TestRegistry& instance()
    {
        static TestRegistry registry;
        return registry;
    }
};

// Test registration helper
struct TestAdder
{
    TestAdder(const std::string& name, TestCase test)
    {
        TestRegistry::instance().tests.emplace_back(name, test);
    }
};

// Macros for defining tests
// line 1: forward declaration
// line 2: test registration
// line 3: test implementation
#define TEST_CASE(name) \
  void name(); \
  TestAdder name##_adder(#name, name); \
  void name()

// boolean assertion
#define ASSERT_TRUE(expr) \
  do { \
    if (!(expr)) { \
      std::cerr << COLOR_RED << "FAIL: " << COLOR_RESET << #expr << " (line " << __LINE__ << ")\n"; \
      throw std::runtime_error("Test failed"); \
    } \
  } while(0)

#define ASSERT_EQUAL(a, b) ASSERT_TRUE((a) == (b))

#define ASSERT_FALSE(expr) \
  do { \
    if ((expr)) { \
      std::cerr << COLOR_RED << "FAIL: " << COLOR_RESET << #expr << " (line " << __LINE__ << ")\n"; \
      throw std::runtime_error("Test failed"); \
    } \
  } while(0)

#define ASSERT_NOT_EQUAL(a, b) ASSERT_FALSE((a) == (b))

// Floating-point comparison with absolute tolerance
#define ASSERT_NEAR(a, b, tolerance) \
  do { \
    double diff = fabs((a) - (b)); \
    if (diff > tolerance) { \
      std::cerr << COLOR_RED << "FAIL: " << COLOR_RESET << #a << " ~= " << #b \
                << " (difference " << diff << " > " << tolerance \
                << ") at line " << __LINE__ << "\n"; \
      throw std::runtime_error("Test failed"); \
    } \
  } while(0)

// Default tolerance version (1e-6)
#define ASSERT_FLOAT_EQUAL(a, b) ASSERT_NEAR(a, b, 1e-6)

#define ASSERT_THROWS(expression) \
    try { \
        expression; \
        std::cout << COLOR_RED << "FAIL: Expected exception, but none thrown." << COLOR_RESET << std::endl; \
    } catch (...) { \
        std::cout << COLOR_BLUE << "PASS: Exception thrown as expected." << COLOR_RESET << std::endl; \
    }

#endif