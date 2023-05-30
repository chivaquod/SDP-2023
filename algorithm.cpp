

#include "algorithm.h"
#include <math.h>

void heart_rate_and_oxygen_saturation(float* pun_ir_buffer, int32_t n_ir_buffer_length, float* pun_red_buffer, float* pn_spo2, int8_t* pch_spo2_valid,
    int32_t* pn_heart_rate, int8_t* pch_hr_valid, float* ratio, float* correl)

{

    int32_t k;   // Declare k as a loop variable
    static int32_t n_last_peak_interval = LOWEST_PERIOD;  // Declare n_last_peak_interval as a static variable, initialized to LOWEST_PERIOD
    float f_ir_mean, f_red_mean, f_ir_sumsq, f_red_sumsq; // Declare variables for storing mean and sum of squares of ir and red signals
    // Declare variables for storing ac values and linear regression coefficients of ir and red signals
    float f_y_ac, f_x_ac, xy_ratio;
    float beta_ir, beta_red, x;
    float an_x[RFA_BUFFER_SIZE], * ptr_x;
    float an_y[RFA_BUFFER_SIZE], * ptr_y;

    // Calculate DC mean of ir and red signals and remove DC from ir and red signals
    f_ir_mean = 0.0;
    f_red_mean = 0.0;
    for (k = 0; k < n_ir_buffer_length; ++k) {
        f_ir_mean += pun_ir_buffer[k];
        f_red_mean += pun_red_buffer[k];
    }
    f_ir_mean = f_ir_mean / n_ir_buffer_length;
    f_red_mean = f_red_mean / n_ir_buffer_length;

    // Remove DC from ir and red signals
    for (k = 0, ptr_x = an_x, ptr_y = an_y; k < n_ir_buffer_length; ++k, ++ptr_x, ++ptr_y) {
        *ptr_x = pun_ir_buffer[k] - f_ir_mean;
        *ptr_y = pun_red_buffer[k] - f_red_mean;
    }

    // Perform linear regression on ir and red signals to remove linear trend (baseline leveling)
    beta_ir = linear_regression_beta(an_x, mean_X, sum_X2);
    beta_red = linear_regression_beta(an_y, mean_X, sum_X2);
    for (k = 0, x = -mean_X, ptr_x = an_x, ptr_y = an_y; k < n_ir_buffer_length; ++k, ++x, ++ptr_x, ++ptr_y) {
        *ptr_x -= beta_ir * x;
        *ptr_y -= beta_red * x;
    }

    // Calculate RMS of IR and RED signals
    f_y_ac = rms(an_y, n_ir_buffer_length, &f_red_sumsq);
    f_x_ac = rms(an_x, n_ir_buffer_length, &f_ir_sumsq);

    // Calculate Pearson correlation between red and IR
    *correl = Pcorrelation(an_x, an_y, n_ir_buffer_length) / sqrt(f_red_sumsq * f_ir_sumsq);

    // Find signal periodicity
    if (*correl >= min_pearson_correlation) {
        // At the beginning of oximetry run the exact range of heart rate is unknown. This may lead to wrong rate if the next call does not find the _first_
        // peak of the autocorrelation function. E.g., second peak would yield only 50% of the true rate. 
        if (LOWEST_PERIOD == n_last_peak_interval)
            initialize_periodicity_search(an_x, RFA_BUFFER_SIZE, &n_last_peak_interval, HIGHEST_PERIOD, min_autocorrelation_ratio, f_ir_sumsq);
        // RF, If correlation os good, then find average periodicity of the IR signal. If aperiodic, return periodicity of 0
        if (n_last_peak_interval != 0)
            signal_periodicity(an_x, RFA_BUFFER_SIZE, &n_last_peak_interval, LOWEST_PERIOD, HIGHEST_PERIOD, min_autocorrelation_ratio, f_ir_sumsq, ratio);
    }
    else n_last_peak_interval = 0;

    // Calculate heart rate if periodicity detector was successful. Otherwise, reset peak interval to its initial value and report error.
    if (n_last_peak_interval != 0) {
        *pn_heart_rate = (int32_t)(FS60 / n_last_peak_interval);
        *pch_hr_valid = 1;
    }
    else {
        n_last_peak_interval = LOWEST_PERIOD;
        // *pn_heart_rate = -999;  //
        *pch_hr_valid = 0;
        //  *pn_spo2 =  -999 ; //
        *pch_spo2_valid = 0;
        //  return; //
    }

    // After trend removal, the mean represents DC level
    xy_ratio = (f_y_ac * f_ir_mean) / (f_x_ac * f_red_mean);
    if (xy_ratio > 0.02 && xy_ratio < 1.84) {
        *pn_spo2 = (-45.060 * xy_ratio + 30.354) * xy_ratio + 94.845;
        *pch_spo2_valid = 1;
    }
    else {
        // *pn_spo2 =  -999 ; 
        *pch_spo2_valid = 0;
    }
}





float linear_regression_beta(float* pn_x, float xmean, float sum_x2)
{
    float x, beta, * pn_ptr;
    beta = 0.0;
    for (x = -xmean, pn_ptr = pn_x; x <= xmean; ++x, ++pn_ptr)
        beta += x * (*pn_ptr);
    return beta / sum_x2;
}

float autocorrelation(float* pn_x, int32_t n_size, int32_t n_lag)

{
    int16_t i, n_temp = n_size - n_lag;
    float sum = 0.0, * pn_ptr;
    if (n_temp <= 0) return sum;
    for (i = 0, pn_ptr = pn_x; i < n_temp; ++i, ++pn_ptr) {
        sum += (*pn_ptr) * (*(pn_ptr + n_lag));
    }
    return sum / n_temp;
}

void initialize_periodicity_search(float* pn_x, int32_t n_size, int32_t* p_last_periodicity, int32_t n_max_distance, float min_aut_ratio, float aut_lag0)

{
    int32_t n_lag;
    float aut, aut_right;
    n_lag = *p_last_periodicity;
    aut_right = aut = autocorrelation(pn_x, n_size, n_lag);
    if (aut / aut_lag0 >= min_aut_ratio) {
        do {
            aut = aut_right;
            n_lag += 2;
            aut_right = autocorrelation(pn_x, n_size, n_lag);
        } while (aut_right / aut_lag0 >= min_aut_ratio && aut_right < aut && n_lag <= n_max_distance);
        if (n_lag > n_max_distance) {
            *p_last_periodicity = 0;
            return;
        }
        aut = aut_right;
    }
    // Walk to the right.
    do {
        aut = aut_right;
        n_lag += 2;
        aut_right = autocorrelation(pn_x, n_size, n_lag);
    } while (aut_right / aut_lag0 < min_aut_ratio && n_lag <= n_max_distance);
    if (n_lag > n_max_distance) {
        *p_last_periodicity = 0;
    }
    else
        *p_last_periodicity = n_lag;
}

void signal_periodicity(float* pn_x, int32_t n_size, int32_t* p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float* ratio)

{
    int32_t n_lag;
    float aut, aut_left, aut_right, aut_save;
    bool left_limit_reached = false;
    n_lag = *p_last_periodicity;
    aut_save = aut = autocorrelation(pn_x, n_size, n_lag);
    aut_left = aut;
    do {
        aut = aut_left;
        n_lag--;
        aut_left = autocorrelation(pn_x, n_size, n_lag);
    } while (aut_left > aut && n_lag >= n_min_distance);
    // Restore lag of the highest aut
    if (n_lag < n_min_distance) {
        left_limit_reached = true;
        n_lag = *p_last_periodicity;
        aut = aut_save;
    }
    else n_lag++;
    if (n_lag == *p_last_periodicity) {
        aut_right = aut;
        do {
            aut = aut_right;
            n_lag++;
            aut_right = autocorrelation(pn_x, n_size, n_lag);
        } while (aut_right > aut && n_lag <= n_max_distance);
        // Restore lag of the highest aut
        if (n_lag > n_max_distance) n_lag = 0;
        else n_lag--;
        if (n_lag == *p_last_periodicity && left_limit_reached) n_lag = 0;
    }
    *ratio = aut / aut_lag0;
    if (*ratio < min_aut_ratio) n_lag = 0;
    *p_last_periodicity = n_lag;
}

float rms(float* pn_x, int32_t n_size, float* sumsq)

{
    int16_t i;
    float r, * pn_ptr;
    (*sumsq) = 0.0;
    for (i = 0, pn_ptr = pn_x; i < n_size; ++i, ++pn_ptr) {
        r = (*pn_ptr);
        (*sumsq) += r * r;
    }
    (*sumsq) /= n_size;
    return sqrt(*sumsq);
}

float Pcorrelation(float* pn_x, float* pn_y, int32_t n_size)

{
    int16_t i;
    float r, * x_ptr, * y_ptr;
    r = 0.0;
    for (i = 0, x_ptr = pn_x, y_ptr = pn_y; i < n_size; ++i, ++x_ptr, ++y_ptr) {
        r += (*x_ptr) * (*y_ptr);
    }
    r /= n_size;
    return r;
}
