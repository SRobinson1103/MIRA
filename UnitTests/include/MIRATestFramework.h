#ifndef MIRA_TEST_FRAMEWORK_H
#define MIRA_TEST_FRAMEWORK_H

#include <vector>
#include <string>
#include <iostream>

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
      std::cerr << "FAIL: " << #expr << " (line " << __LINE__ << ")\n"; \
      throw std::runtime_error("Test failed"); \
    } \
  } while(0)

#define ASSERT_EQUAL(a, b) ASSERT_TRUE((a) == (b))

// Floating-point comparison with absolute tolerance
#define ASSERT_NEAR(a, b, tolerance) \
  do { \
    double diff = fabs((a) - (b)); \
    if (diff > tolerance) { \
      std::cerr << "FAIL: " << #a << " ~= " << #b \
                << " (difference " << diff << " > " << tolerance \
                << ") at line " << __LINE__ << "\n"; \
      throw std::runtime_error("Test failed"); \
    } \
  } while(0)

// Default tolerance version (1e-6)
#define ASSERT_FLOAT_EQUAL(a, b) ASSERT_NEAR(a, b, 1e-6)

#endif