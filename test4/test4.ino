#include "RobotConfig.h"

// ==========================================
// KHAI BÁO BIẾN ĐIỀU KHIỂN CỦA BẠN
// ==========================================
int chutich = 0;
int mn = 0; 
bool nv1 = true;
bool nv2 = false;
bool nv3 = false;
bool nv4 = false;

// ==========================================
// HÀM NGẮT ĐỌC ENCODER (Để nguyên không đụng chạm)
// ==========================================
void ISR_FL() { if (digitalRead(FL_ENCB)) count_FL--; else count_FL++; }
void ISR_FR() { if (digitalRead(FR_ENCB)) count_FR--; else count_FR++; }
void ISR_RL() { if (digitalRead(RL_ENCB)) count_RL--; else count_RL++; }
void ISR_RR() { if (digitalRead(RR_ENCB)) count_RR--; else count_RR++; }

void setup() {
    Serial.begin(115200);

    ADCSRA = (ADCSRA & 0xF8) | 0x05; 

    dung_xe_ngay();
    cali_5_giay();
    
    // Cài đặt chân ngắt Encoder
    pinMode(FL_ENCA, INPUT_PULLUP); pinMode(FL_ENCB, INPUT_PULLUP);
    pinMode(FR_ENCA, INPUT_PULLUP); pinMode(FR_ENCB, INPUT_PULLUP);
    pinMode(RL_ENCA, INPUT_PULLUP); pinMode(RL_ENCB, INPUT_PULLUP);
    pinMode(RR_ENCA, INPUT_PULLUP); pinMode(RR_ENCB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(FL_ENCA), ISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENCA), ISR_FR, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENCA), ISR_RL, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENCA), ISR_RR, RISING);

    dung_xe_ngay();
    delay(1000); // Chờ 1 giây cho xe ổn định trước khi xuất phát
}
int tinh_do_gan(int a, int b) {
    // Nếu là cặp đặc biệt (2, 4)
    if ((a == 2 && b == 4) || (a == 4 && b == 2)) {
        // Trả về 15 (Thua khoảng cách 1 thật, nhưng thắng khoảng cách 2)
        return 15; 
    }
    // Các trường hợp khác tính khoảng cách bình thường rồi nhân 10
    return abs(a - b) * 10;
}
// int hang1 = 0;
// int hang2 = 0;
// int hang3 = 0;
// int hang4 = 0;
// // 1 đỏ 2 vàng 3 cam 4 xanh lá 5 xanh biển
// void giaohangthongminh(int mau1, int mau2){
//     // Bước 1: Dọn dẹp chuỗi (xóa sạch khoảng trắng và ngoặc tròn)
//     chuoi_nhan.replace(" ", "");
//     chuoi_nhan.replace("(", "");
//     chuoi_nhan.replace(")", "");
//     // Lúc này chuỗi sẽ sạch sẽ mượt mà: "hang1:15|hang2:0|hang3:12|hang4:5"

//     // Bước 2: Băm chuỗi bằng vòng lặp
//     while (chuoi_nhan.length() > 0) {
//         int vi_tri_pipe = chuoi_nhan.indexOf('|');
//         String cum_lenh = "";

//         if (vi_tri_pipe == -1) {
//             cum_lenh = chuoi_nhan; // Không còn dấu '|' -> Đây là đơn hàng cuối cùng
//             chuoi_nhan = "";       // Xóa chuỗi để thoát vòng lặp
//         } else {
//             cum_lenh = chuoi_nhan.substring(0, vi_tri_pipe); // Cắt lấy "hang1:15"
//             chuoi_nhan = chuoi_nhan.substring(vi_tri_pipe + 1); // Giữ lại phần đuôi
//         }

//         // Bước 3: Tách Tên Hàng và Giá Trị từ "hang1:15"
//         int vi_tri_hai_cham = cum_lenh.indexOf(':');
//         if (vi_tri_hai_cham > 0) {
//             String ten_hang = cum_lenh.substring(0, vi_tri_hai_cham);
//             int gia_tri = cum_lenh.substring(vi_tri_hai_cham + 1).toInt();

//             // Bước 4: Lưu giá trị vào đúng biến kho
//             // Hỗ trợ Pi 5 gửi chữ "hang1" hoặc viết tắt "h1" đều hiểu
//             if (ten_hang == "hang1" || ten_hang == "h1") {
//                 hang1 = gia_tri;
//             } 
//             else if (ten_hang == "hang2" || ten_hang == "h2") {
//                 hang2 = gia_tri;
//             }
//             else if (ten_hang == "hang3" || ten_hang == "h3") {
//                 hang3 = gia_tri;
//             }
//             else if (ten_hang == "hang4" || ten_hang == "h4") {
//                 hang4 = gia_tri;
//             }
//         }
//     }
//     hang1
//     hang2
//     hang3
//     hang4

// }
void giaohang(){
    can_chinh_dau_xe();
    bam_line_den_goc_phai(70.0);
    bam_line_khoang_cach(50.0, 180);
    dung_xe_ngay();
    delay_pid(1500); //##############################RED
    lui_khoang_cach(50.0, 100);
    
    truot_ngang_phai_tim_vach(50.0);
    bam_line_khoang_cach(40.0, 100);
    dung_xe_ngay();
    delay_pid(1500);
    lui_khoang_cach(45.0,100);

   /* xoay_tai_cho_pid(1);
    bam_line_den_nga_tu(45.0);
    tien_khoang_cach(45.0, 150);    

    bam_line_den_goc_trai(45.0);
    xoay_thong_minh(-1);*/ 
    truot_ngang_phai_tim_vach(50.0);//#########
    dung_xe_ngay();
    delay_pid(500);       // Dừng xe chờ 0.5 giây
    can_chinh_dau_xe();
    lui_khoang_cach(45.0,40);

    truot_ngang_phai_tim_vach(50.0);//#########
    dung_xe_ngay();
    delay_pid(1000);       // Dừng xe chờ 0.5 giây
    can_chinh_dau_xe();
    
    bam_line_khoang_cach(40.0, 120);
    dung_xe_ngay();
    delay_pid(1500); 
    lui_khoang_cach(45.0, 100);
    
    truot_ngang_phai_tim_vach(50.0);
    dung_xe_ngay();
    delay_pid(500);       // Dừng xe chờ 0.5 giây
    can_chinh_dau_xe();
    bam_line_khoang_cach(45.0, 100); 
    lui_khoang_cach(40.0,80);
}

void layhang(){
    // ========================================================
    // KHỐI 1: CHUỖI LỆNH CHUNG (Chỉ chạy khi không phải nhiệm vụ 4)
    // ========================================================
    if (nv4 == false && nv3 == false) {
        xoay_tai_cho_pid(-1);
        dung_xe_ngay(); delay_pid(500);
        can_chinh_dau_xe(); 
        bam_line_den_khi_mat_vach(60);
        tien_khoang_cach(50.0, 100);
        dung_xe_ngay(); delay_pid(500);
        xoay_tai_cho_pid(-1);
        bam_line_den_goc_trai(50.0);
    }

    // ========================================================
    // KHỐI 2: KỊCH BẢN RIÊNG CHO TỪNG NHIỆM VỤ DỰA VÀO BIẾN TRUE/FALSE
    // ========================================================
    
    // Khi đang chạy NV1 (nv1 đã bật true trong hàm nv1xe, nhưng nv2 vẫn false)
    if (nv1 == false) {
        xoay_thong_minh(-1);
        bam_line_den_nga_tu(50.0);
        tien_khoang_cach(50.0, 120); 
        bam_line_den_nga_tu(50.0);
        xoay_thong_minh(1);
        bam_line_khoang_cach(40.0, 100);
    }
    // Khi đang chạy NV2 (nv2 đã bật true, nhưng nv3 vẫn false)
    else if (nv2 == false) {
        dung_xe_ngay(); delay_pid(500);
        xoay_thong_minh(-1); 
        bam_line_den_nga_tu(50.0);
        dung_xe_ngay(); delay_pid(300);
        xoay_thong_minh(1);
        dung_xe_ngay(); delay_pid(500);
        can_chinh_dau_xe();
        bam_line_khoang_cach(40.0, 120); 
    }
    // Khi đang chạy NV3 (nv3 đã bật true, nhưng nv4 vẫn false)
    else if (nv3 == false) {
        bam_line_khoang_cach(50.0, 330); 
        dung_xe_ngay(); delay_pid(1500); 
        lui_khoang_cach(50.0, 195);
        dung_xe_ngay(); delay_pid(300); 
    }
    // Khi đang chạy NV4 (nv4 đã được bật true) -> ĐÂY CHÍNH LÀ ĐOẠN ELSE CỦA BẠN!
    else if (nv4 == false) {
        truot_ngang_trai_tim_vach(50);
        dung_xe_ngay(); delay_pid(300); // Trượt xong phải cho xe nghỉ
        xoay_tai_cho_pid(1); 
        can_chinh_dau_xe();
        bam_line_den_nga_tu(50.0);
        bam_line_khoang_cach(50.0, 150);
        dung_xe_ngay(); delay_pid(1000);
        //##############################################
        dung_xe_ngay(); delay_pid(1000);
        lui_khoang_cach(50.0, 200);
        dung_xe_ngay(); delay_pid(300);
        xoay_tai_cho_pid(1);
        bam_line_den_nga_tu(50.0);
        xoay_thong_minh_pid(1);
        bam_line_khoang_cach(50.0, 200);
        lui_khoang_cach(50.0, 200);
        xoay_tai_cho_pid(1);
        bam_line_den_nga_tu(50.0);
        xoay_thong_minh_pid(1);
        bam_line_khoang_cach(50.0, 100);
        tien_khoang_cach(70.0, 200);
    }
}


// ==========================================
// KỊCH BẢN CHẠY XE (DỄ HIỂU - TRỰC QUAN)
// ==========================================
void nv1xe() {
    nv1 = true;
    // BƯỚC 1: Bám line đến góc chữ L phải -> Bỏ qua nó và tiếp tục bám vạch thêm 330mm
    bam_line_den_goc_phai(50.0); 
    bam_line_khoang_cach(40.0, 330); 
    
    // BƯỚC 2: Dừng khựng lại 1500ms để giả lập gắp hàng
    dung_xe_ngay();
    delay_pid(1500); 
    
    // BƯỚC 3: Lùi thẳng 195mm về lại đúng tâm giao điểm -> Xoay phải 90 độ
    lui_khoang_cach(50.0, 195);
    delay_pid(200);      // Nghỉ 0.2s triệt tiêu quán tính
    xoay_tai_cho_pid(1); // Số 1 là rẽ Phải, Số 0 là rẽ Trái
    
    // BƯỚC 4: Bám line đến ngã tư bự -> Nhắm mắt nhào tới 150mm để xuyên qua vùng đen
    bam_line_den_nga_tu(70.0);
    tien_khoang_cach(70.0, 120); 
    
    // BƯỚC 5: Bám line đến ngã tư tiếp theo -> Tiến bụng xe vào tâm ngã tư 135mm -> Xoay phải
    bam_line_den_nga_tu(70.0);
    tien_khoang_cach(70.0, 100);   // Nghỉ 0.2s 
    xoay_tai_cho_pid(1); // Xoay Phải
    

    giaohang();
    layhang();
    // KẾT THÚC BÀI THI
    chutich = 1; 
}
void nv2xe() {
    nv2 = true;
    dung_xe_ngay();
    delay_pid(1500); 
    
    // BƯỚC 3: Lùi thẳng 195mm về lại đúng tâm giao điểm -> Xoay phải 90 độ
    lui_khoang_cach(50.0, 125);
    delay_pid(200);      // Nghỉ 0.2s triệt tiêu quán tính
    xoay_tai_cho_pid(1); // Số 1 là rẽ Phải, Số 0 là rẽ Trái
    
    bam_line_den_nga_tu(50.0);
    tien_khoang_cach(50.0, 135);
    delay_pid(200);      // Nghỉ 0.2s 
    xoay_tai_cho_pid(1); // Xoay Phải

    giaohang();
    layhang();
    // KẾT THÚC BÀI THI
    chutich = 3; 
}
void nv3xe(){
    dung_xe_ngay();
    delay_pid(1500); 
    
    // BƯỚC 3: Lùi thẳng 195mm về lại đúng tâm giao điểm -> Xoay phải 90 độ
    xoay_tai_cho_pid(1);
    nv3 = true;
    giaohang();
    layhang();
    chutich = 4; 
}

void nv4xe(){
    while(true){
        dung_xe_ngay();
    }
}

// ==========================================
// VÒNG LẶP CHÍNH CỦA XE (TRÁI TIM HỆ THỐNG)
// ==========================================
void loop() {
    // 1. BẮT BUỘC: Nuôi sống bộ đếm gia tốc và thăng bằng động cơ liên tục

    cap_nhat_pid_dong_co();

    // 2. Phân luồng kịch bản
    switch (chutich) {
        case 0:
            nv1xe();
            break;
            
        case 1: 
            nv2xe();
            break;
        case 3:
            nv3xe();
            break;
        case 4:{
            nv4xe();
            break;
        }
        case 5:{
            dung_xe_ngay();
        }
    }   
}