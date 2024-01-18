#include "gtest/gtest.h"
#include "mpc_lateral_controller/lowpass_filter.hpp"

#include <vector>

TEST(TestLowpassFilter, MoveAverageFilter) // Case-1
{
  namespace MoveAverageFilter =
    autoware::motion::control::mpc_lateral_controller::MoveAverageFilter;

  {  // Fail case: window size higher than the vector size
    const int window_size = 5;
    std::vector<double> vec = {1.0, 2.0, 3.0, 4.0};
    EXPECT_FALSE(MoveAverageFilter::filt_vector(window_size, vec)); // checks that the specified condition evaluates to false.

  }
  // Succeeds with a window size of 0,1,2,3,4, applying the moving average filter:
  {
    const int window_size = 0;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec)); // checks that the specified condition evaluates to true.
    ASSERT_EQ(filtered_vec.size(), original_vec.size()); // checks that two values are equal.
    for (size_t i = 0; i < filtered_vec.size(); ++i) {
      EXPECT_EQ(filtered_vec[i], original_vec[i]); // checks that two values are equal.
    }
  }
  {
    const int window_size = 1;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 13.0 / 3);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int window_size = 2;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]); // Checks if the first element of the filtered vector is equal to the first element of the original vector.
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 21.0 / 5);
    EXPECT_EQ(filtered_vec[3], 30.0 / 5);
    EXPECT_EQ(filtered_vec[4], 23.0 / 3);
    EXPECT_EQ(filtered_vec[5], original_vec[5]);
  }
  {
    const int window_size = 3;
    const std::vector<double> original_vec = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 1.0);
    EXPECT_EQ(filtered_vec[2], 1.0);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int window_size = 4;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 21.0 / 5);
    EXPECT_EQ(filtered_vec[3], 30.0 / 5);
    EXPECT_EQ(filtered_vec[4], 23.0 / 3);
    EXPECT_EQ(filtered_vec[5], original_vec[5]);
  }
}
TEST(TestLowpassFilter, Butterworth2dFilter) // Case-2
{
  using autoware::motion::control::mpc_lateral_controller::Butterworth2dFilter;
  const double dt = 1.0;
  const double cutoff_hz = 1.0;
  Butterworth2dFilter filter(dt, cutoff_hz); //  initializes a Butterworth filter with a given sampling time dt and cutoff frequency cutoff_hz.
  for (double i = 1.0; i < 10.0; ++i) {
    EXPECT_LT(filter.filter(i), i); //  iterates over increasing input values to check if the filtered values are less than the original values.
  }

  const std::vector<double> original_vec = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> filtered_vec;
  filter.filt_vector(original_vec, filtered_vec); // Applies the Butterworth filter to the input vector original_vec and stores the filtered result in filtered_vec.
  ASSERT_EQ(filtered_vec.size(), original_vec.size());
  EXPECT_NEAR(filtered_vec[0], original_vec[0], 1.0e-10); // checks that two values are approximately equal within a certain tolerance (1.0e-10 in this case).
  for (size_t i = 1; i < filtered_vec.size(); ++i) {
    EXPECT_LT(filtered_vec[i], original_vec[i]); //  checks that the first value is less than the second value.
  }

  filtered_vec.clear();
  filter.filtfilt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size()); // Asserts that the size of the filtered vector is equal to the size of the original vector.

  std::vector<double> coefficients;
  filter.getCoefficients(coefficients); //  retrieves the filter coefficients.
  // Expects that the size of the coefficients vector is 5. This ensures that the Butterworth filter has the expected number of coefficients.
  EXPECT_EQ(coefficients.size(), size_t(5));
}

// Comparison of the coefficients
// These tests involve comparing the coefficients of the Butterworth filter for different cases.
TEST(TestLowpassFilter, Butterworth2dFilterCoeffs) // Case-3
{
  using autoware::motion::control::mpc_lateral_controller::Butterworth2dFilter;

  // Case 1:
  // cutoff_frequency = 1.0 [Hz], sampling_time = 0.033
  //
  //   0.0093487 +0.0186974z +0.0093487z²
  //   ----------------------------------
  //       0.7458606 -1.7084658z +z²
  // The specific numbers (coef values) in the numerator and denominator of above equation are calculated based on the desired cutoff frequency and sampling time.
  {
    const double sampling_time = 0.033;
    const double f_cutoff_hz = 1.0;
    Butterworth2dFilter butt_filter(sampling_time, f_cutoff_hz);
    std::vector<double> coeff;
    butt_filter.getCoefficients(coeff);
    constexpr double ep = 1.0e-6;
    EXPECT_NEAR(coeff.at(0), 1.7084658, ep);   // a1 //  Expects the first coefficient (a1) to be approximately 1.7084658.
    EXPECT_NEAR(coeff.at(1), -0.7458606, ep);  // a2
    EXPECT_NEAR(coeff.at(2), 0.0093487, ep);   // b0
    EXPECT_NEAR(coeff.at(3), 0.0186974, ep);   // b1
    EXPECT_NEAR(coeff.at(4), 0.0093487, ep);   // b2
  }

  // Case 1:
  // cutoff_frequency = 2.0 [Hz], sampling_time = 0.05
  //
  //    0.0674553 +0.1349105z +0.0674553z²
  //    ----------------------------------
  //        0.4128016 -1.1429805z +z²

  // The coefficients are recalculated based on the new desired cutoff frequency and sampling time.
  {
    const double sampling_time = 0.05;
    const double f_cutoff_hz = 2.0;
    Butterworth2dFilter butt_filter(sampling_time, f_cutoff_hz);
    std::vector<double> coeff;
    butt_filter.getCoefficients(coeff);
    constexpr double ep = 1.0e-6;
    EXPECT_NEAR(coeff.at(0), 1.1429805, ep);   // a1 // Expects the first coefficient (a1) to be approximately 1.1429805.
    EXPECT_NEAR(coeff.at(1), -0.4128016, ep);  // a2
    EXPECT_NEAR(coeff.at(2), 0.0674553, ep);   // b0
    EXPECT_NEAR(coeff.at(3), 0.1349105, ep);   // b1
    EXPECT_NEAR(coeff.at(4), 0.0674553, ep);   // b2
  }
}
