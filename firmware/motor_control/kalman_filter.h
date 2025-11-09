#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float x;  // 狀態估計
    float P;  // 估計誤差協方差
    float Q;  // 過程噪聲
    float R;  // 測量噪聲
} kalman_t;

void kalman_init(kalman_t* kf, float Q, float R);
float kalman_update(kalman_t* kf, float measurement);

#endif // KALMAN_FILTER_H
