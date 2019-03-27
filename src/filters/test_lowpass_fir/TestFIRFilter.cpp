
#include "filter/FIRFilter.hpp"
#include "matvec.hpp"
#include "gtest/gtest.h"

namespace {

TEST(FIRFilter, DefaultConstructorFloat)
{
    am2b::FIRFilter<float> filter;
}

TEST(FIRFilter, InitFilter)
{
    const double coeffs[] = { 0.5, 0.4, 0.3, 0.2, 0.1 };
    am2b::FIRFilter<float> filter;
    filter.init(5, coeffs);

    EXPECT_FLOAT_EQ(filter.getOutput(), 0.0);
}

TEST(FIRFilter, FilterStepVector)
{
    // prototype vector
    am2b::vec inputVector(2);

    const double coeffs[] = { 0.5, 0.4, 0.3, 0.2, 0.1 };
    am2b::FIRFilter<am2b::vec> filter(inputVector);
    filter.init(5, coeffs);

    EXPECT_FLOAT_EQ(filter.getOutput()(0), 0.0);
    EXPECT_FLOAT_EQ(filter.getOutput()(1), 0.0);

    inputVector(0) = 1.0;
    inputVector(1) = 3.0;

    filter.update(inputVector);

    // Expected result for digital filter after one step
    EXPECT_FLOAT_EQ(filter.getOutput()(0), 0.5);
    EXPECT_FLOAT_EQ(filter.getOutput()(1), 1.5);
}

TEST(FIRFilter, MatlabRefStepResponse)
{

    const int BL = 12;
    const double B[12] = {
        -0.001451684596776, 0.007880498503156, -0.02628524668337, 0.06936959978317,
        -0.1714599686352, 0.6218885929298, 0.6218885929298, -0.1714599686352,
        0.06936959978317, -0.02628524668337, 0.007880498503156, -0.001451684596776
    };

    // The expected filter values were calculated with matlab
    const double expected[12]{ -0.0014516845967762178, 0.0064288139063801318, -0.019856432776985088,
        0.049513167006185688, -0.12194680162900916, 0.49994179130081567, 1.1218303842306405,
        0.95037041559544566, 1.0197400153786165, 0.99345476869525118, 1.0013352671984077,
        0.99988358260163124 };

    am2b::FIRFilter<double> filter;
    filter.init(BL, B);

    EXPECT_FLOAT_EQ(filter.getOutput(), 0.0);

    for (int i = 0; i < BL; i++) {

        filter.update(1.0);
        EXPECT_FLOAT_EQ(filter.getOutput(), expected[i]);
    }
}
}
