
#include <Arduino.h>
#define ST 4	// Sampling time in s.  
#define FS 50	// Sampling frequency in Hz.  

const float sum_X2 = 666650.0;	// Pre-calculated sum of squares of ST *FS numbers
#define MAX_HR 180 	// Maximum heart rate to eliminate erroneous signals
#define MIN_HR 50	// 

const float min_autocorrelation_ratio = 0.5;	// Minimum ratio of two autocorrelation sequence elements

const float min_pearson_correlation = 0.8;	// Minimum Pearson correlation between red and IR signals

const int32_t RFA_BUFFER_SIZE = FS * ST;	// Number of smaples in a single batch
const int32_t FS60 = FS * 60;	// Conversion factor for heart rate from bps to bpm
const int32_t LOWEST_PERIOD = FS60 / MAX_HR;	// Minimal distance between peaks
const int32_t HIGHEST_PERIOD = FS60 / MIN_HR;	// Maximal distance between peaks
const float mean_X = (float)(RFA_BUFFER_SIZE - 1) / 2.0;	// Mean value of the set of integers from 0 to RFA_BUFFER_SIZE-1. For ST=4 and FS=50 it's equal to 99.5.

void heart_rate_and_oxygen_saturation(float* pun_ir_buffer, int32_t n_ir_buffer_length, float* pun_red_buffer, float* pn_spo2, int8_t* pch_spo2_valid, int32_t* pn_heart_rate,
	int8_t* pch_hr_valid, float* ratio, float* correl);
float linear_regression_beta(float* pn_x, float xmean, float sum_x2);
float autocorrelation(float* pn_x, int32_t n_size, int32_t n_lag);
float rms(float* pn_x, int32_t n_size, float* sumsq);
float Pcorrelation(float* pn_x, float* pn_y, int32_t n_size);
void initialize_periodicity_search(float* pn_x, int32_t n_size, int32_t* p_last_periodicity, int32_t n_max_distance, float min_aut_ratio, float aut_lag0);
void signal_periodicity(float* pn_x, int32_t n_size, int32_t* p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float* ratio);

