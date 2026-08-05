#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <AFMotor.h>

// Định nghĩa chân ngắt Encoder (Dành cho Arduino Mega 2560)
#define FL_ENCA 18
#define FL_ENCB 26
#define FR_ENCA 20
#define FR_ENCB 23
#define RL_ENCA 19
#define RL_ENCB 24
#define RR_ENCA 21
#define RR_ENCB 25

#define XUNG_1_VONG 495

enum TrangThaiLine {
  LINE_LOST,
  LINE_NORMAL,
  LINE_ALL_BLACK
};

// Khai báo Motor
extern AF_DCMotor M1;
extern AF_DCMotor M2;
extern AF_DCMotor M3;
extern AF_DCMotor M4;

// Khai báo biến toàn cục
extern volatile long count_FL, count_FR, count_RL, count_RR;
extern bool m1, m2, m3, m4, m5, m6, m7, m8;
extern float filteredSensor[8];
extern int base_speed;
extern float lineError;
extern TrangThaiLine trangThaiLineHienTai;

// Biến cho kịch bản
extern int chutich;
extern int mn;

// ==============================================
// KHAI BÁO TẤT CẢ CÁC HÀM SỬ DỤNG
// ==============================================
void cap_nhat_pid_dong_co();
void delay_pid(unsigned long ms);
void setTargetRPM(float rf, float rr, float lf, float lr);
void reset_encoder();
void doc_tat_ca_encoder(long &rf, long &rr, long &lf, long &lr);
long doi_mm_ra_xung(long khoangCachMM);

void tien_truc_tiep(float rpm);
void lui_truc_tiep(float rpm);
void quay_trai_truc_tiep(float rpm);
void quay_phai_truc_tiep(float rpm);
void dung_xe_ngay();
void dung_tat_ca_dong_co();
void tien(int speed);
void lui(int speed);
void trai(int speed);
void phai(int speed);
void dung();

void rd();
TrangThaiLine tinh_sai_so_line(float &error);
bool phat_hien_mau_goc_phai_90();
bool phat_hien_mau_goc_trai_90();
void dat_lai_bo_bam_line();
void bam_line_pid(float tocDoCoSo);

void tien_khoang_cach(float toc_do, long mm);
void lui_khoang_cach(float toc_do, long mm);
void bam_line_khoang_cach(float toc_do, long mm);
void bam_line_den_nga_tu(float toc_do);
void bam_line_den_goc_phai(float toc_do);
void bam_line_den_goc_trai(float toc_do);
void xoay_tai_cho_pid(int huong);
void xoay_thong_minh_pid(int huong);
void xoay_thong_minh(int huong);

void di_chuyen_vong(float so_vong, int target_speed, int dir_FL, int dir_FR, int dir_RL, int dir_RR);
void toivong(float so_vong, int speed);
void chay_ngang_encoder(int target_speed, int huong);
void luithang_encoder(int target_speed);
void xoay(int huong, int speed_nhanh, int speed_cham);
void chayngang(int huong, float rpm);
void truot_ngang_phai_tim_vach(float toc_do);
void truot_ngang_trai_tim_vach(float toc_do);

// Hàm Cali có đầy đủ dấu chấm phẩy ở cuối
void cali_5_giay();
void bam_line_den_khi_mat_vach(float toc_do);
void can_chinh_dau_xe();

#endif