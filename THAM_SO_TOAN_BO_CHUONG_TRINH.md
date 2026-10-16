# Thông số xe và tham số mô phỏng

Tài liệu tổng hợp các giá trị mặc định và cấu hình được mã hóa trong các chương trình Python, tập lệnh chạy và hướng dẫn của dự án. Tham số dòng lệnh truyền khi chạy có thể ghi đè giá trị mặc định. Các thông số vật lý thực tế của xe CARLA được đọc tại lúc chạy, không được cố định trong mã nguồn; xem `tham_so.json` hoặc `tham_so_carla.json` trong thư mục kết quả của lượt chạy để lấy số đo cụ thể.

## 1. Xe mô phỏng

| Thuộc tính | Giá trị / cách lấy |
|---|---|
| Blueprint mặc định | `vehicle.tesla.model3` |
| Số bánh yêu cầu | 4; chương trình dừng nếu blueprint không có đúng bốn bánh |
| Khối lượng | Đọc từ `get_physics_control().mass` (kg); các bài cơ bản nhân với `--mass-scale` (mặc định 1.0) |
| Chiều dài cơ sở | Tính từ tọa độ bánh trước/sau và kiểm tra tương quan với bounding box; `--wheelbase` cho phép cung cấp giá trị đã xác minh (m) |
| Góc lái bánh trước tối đa | Đọc cho từng bánh từ CARLA `max_steer_angle` |
| Ma sát lốp | Lấy từ từng bánh; có thể đặt bằng `--tire-friction` trong các chế độ pad/bài maneuver |
| Thông số vật lý khác | Khi chạy thời gian thực, báo cáo khối lượng, drag coefficient, tâm khối lượng, max RPM, mô men quán tính, final ratio, steering curve, và thông số từng bánh (ma sát, damping, bán kính, mô men phanh, góc lái tối đa, độ cứng ngang...) vào `tham_so_carla.json` |

**Lưu ý:** đây là blueprint xe CARLA, không phải thông số đã xác nhận của xe thật. Trọng lượng, chiều dài cơ sở, mô men quán tính và góc lái không thể kết luận chỉ từ tên blueprint; phải đọc báo cáo lượt chạy. Mặc định chỉ thay đổi khối lượng theo hệ số 1.0, không hiệu chỉnh hàng loạt thông số xe.

## 2. Kết nối và thế giới CARLA

| Tham số | Mặc định |
|---|---:|
| Host | `127.0.0.1` |
| RPC port | `2000` |
| RPC timeout | 90 s ở các bài thu dữ liệu/maneuver/workflow; 30 s ở mô phỏng thời gian thực; tùy công cụ kiểm tra có thể là 15 s |
| Bản CARLA được hướng dẫn | 0.9.15 |
| Bản đồ/đường thử | Đường OpenDRIVE tạo trong chương trình; chế độ thời gian thực pad tạo pad phẳng và đặt xe tại đường thử |
| Thời tiết mặc định | `ClearNoon`; `--rain-visual` bật `WetNoon`/hiệu ứng mưa nhìn thấy được ở công cụ hỗ trợ |
| Đồng bộ thời gian | `synchronous_mode=True`, bước cố định `fixed_delta_seconds=dt` |
| Substepping vật lý | Bật; `max_substep_delta_time=0.005 s`, `max_substeps=10` (ở bộ thu và mô phỏng thời gian thực) |
| Kiểm tra bước thời gian | Sai lệch cho phép tối đa 1% của dt (hoặc 1 micro giây nếu lớn hơn) |

Các chế độ `create`, `auto`, `reuse` quyết định tạo mới, tự chọn hoặc dùng lại thế giới/đường; mặc định của bộ thu là `create`, của bài ngang riêng là `auto`. Quan sát (`--observe`) đồng bộ tốc độ mô phỏng với thời gian thực, bật camera bám xe và giữ xe đóng băng sau thu thập.

## 3. Thu dữ liệu cơ bản (`buoc1_carla.py`)

| Tham số | Mặc định |
|---|---:|
| Bước thời gian `dt` | 0.02 s (50 Hz) |
| Thời lượng | 18 s; 900 khoảng bước, tối đa 901 mẫu tùy cách tính mẫu đầu/cuối |
| Tốc độ mục tiêu bài ngang | 2.0 m/s (7.2 km/h) |
| Ga dọc | 0.18 (chuẩn hóa) |
| Biên độ lái | 0.06 (lệnh `VehicleControl.steer` chuẩn hóa, không phải rad/độ) |
| Tần số lái hình sin | 0.2 Hz |
| Ngưỡng cắt tốc độ | 10.0 m/s |
| Hệ số khối lượng | 1.0 |
| Ổn định ban đầu | Phanh 1.0 trong 2.0 s |
| Điều kiện tham số | `0 < dt <= 0.05`; tốc độ dương không vượt ngưỡng; ga `0..0.5`; `|steer amplitude| <= 0.15`; tần số dương |

Các case: `doc`, `ngang`, `phoi_hop`. Lái hình sin dùng trong phần kết hợp. Case bước lái tăng dần trong 0.5 s từ giây 5, giữ đến giây 11 rồi nhả; không có bộ điều khiển bám quỹ đạo. Với case phối hợp, tốc độ mục tiêu có thể tăng lên `1.25 × speed` theo pha lệnh.

## 4. Bài thử maneuver (`kiem_thu_maneuver.py`)

| Tham số | Mặc định / lựa chọn |
|---|---|
| Maneuver | Bắt buộc: `jturn` hoặc `double_pulse` |
| Tốc độ vào | Bắt buộc chọn 30, 50 hoặc 80 km/h |
| Nhãn ma sát | Bắt buộc chọn 1.0 (khô) hoặc 0.4 (ướt) |
| Ma sát bánh CARLA | Bắt buộc truyền `--tire-friction` |
| Góc bánh trước mục tiêu | 2° (không phải góc vô lăng) |
| Hướng đánh lái | +1; được chọn ±1 |
| Xung phanh | 0.0; lệnh chuẩn hóa trong khoảng pha 2.0–2.5 s nếu bật |
| dt | 0.01 s (100 Hz) |
| Ổn định ban đầu | 2 s phanh, sau đó tối đa 40 s điều chỉnh tốc độ vào |
| Thời lượng maneuver | 6 s sau khi đạt tốc độ vào (dt=0.01 s, khoảng 601 mẫu) |
| Tốc độ tối đa cắt lượt | 35 m/s (126 km/h) |
| Quan sát | Tùy chọn; pacing theo thời gian thực, camera bám xe, giữ xe đóng băng cuối lượt |

`mu-label` chỉ là nhãn điều kiện thí nghiệm; ma sát thực tế áp dụng cho lốp được lưu riêng. `rain-visual` chỉ thay đổi hiển thị thời tiết và không tự thay thế tham số ma sát lốp.

## 5. Ma trận và workflow

- `chay_ma_tran.py`: lặp cấu hình khô/ướt do người dùng truyền vào; mặc định 3 lần mỗi tổ hợp và hướng `[+1]`; góc lái 2°; xe Tesla Model 3; mặc định chỉ lập kế hoạch, cần `--execute` để chạy CARLA. `--directions` nhận ±1, có thể tạo số lượt theo tích số điều kiện × hướng × số lần lặp.
- `chay_quy_trinh.py`: workflow `smoke`, `baseline` hoặc `matrix`; khô 1.0, ướt 0.4, 3 lần lặp, xe Tesla Model 3, host/port 127.0.0.1:2000, timeout 90 s.
- Các bài kiểm chứng tiếp diễn (`chay_tiep_kiem_chung.py`) lấy đường dẫn baseline bắt buộc; mặc định xe Tesla, timeout 90 s, world mode `create`.
- Thời tiết mưa là tùy chọn hiển thị; hệ số ma sát lốp được truyền riêng.

## 6. Mô phỏng thời gian thực và mô hình bóng (`mo_phong_thoi_gian_thuc.py`)

### Thiết lập mặc định

| Tham số | Mặc định |
|---|---:|
| Mode | `pad` (hoặc `attach` để gắn vào xe đang chạy) |
| dt | 0.01 s (100 Hz), giới hạn cho phép `<= 0.02 s` |
| Thời lượng | 60 s mô phỏng; 0 nghĩa là chạy đến Ctrl+C |
| Tốc độ mục tiêu | 30 km/h (phạm vi hợp lệ 5–100 km/h) |
| Maneuver | `double_pulse`; tùy chọn `jturn`, `double_pulse`, `none` |
| Góc bánh trước | 2° (phạm vi `0 < delta <= 10°`) |
| Hướng | +1 (cho phép ±1) |
| Lặp maneuver | Tắt (`repeat-s=0`); nếu bật, đổi bên luân phiên |
| Ma sát lốp | Không ghi đè nếu không truyền; nếu truyền, phạm vi `(0, 10]` |
| Cập nhật tham số xe | 1 s |
| Reset khởi tạo mô hình bóng | 1 s (phạm vi 0.2–10 s) |
| Tần số log console | 5 Hz |
| Cửa sổ đồ thị | 20 s; biểu đồ tắt mặc định |
| Chờ xe ở attach mode | 60 s; role name `hero`; có thể chọn actor id |
| CoM đến trục trước / wheelbase | 0.45 giả định nếu không có telemetry; phạm vi (0,1) |
| Mô hình bóng | Bật mặc định; có thể tắt bằng `--no-model` |
| Telemetry | Đọc nếu có, trừ khi `--no-telemetry`; tự chọn nguồn khi báo cáo |
| Tốc độ chạy | Theo thời gian thực; `--fast` bỏ pacing (pad mode) |

### Nhận dạng mô hình

Mô hình động học và động lực học dùng trục thân xe X tiến/Y trái, yaw và lái trái dương; đơn vị SI. Chiều dài cơ sở và khối lượng lấy trực tiếp từ CARLA. Độ cứng lốp trước/sau, phân bố vị trí trọng tâm và mô men quán tính quanh trục yaw được nhận dạng online từ trạng thái/telemetry. Khi không đủ telemetry, các giả định/nguồn ước lượng được ghi trong `_sources` của `tham_so.json`; mô men quán tính có thể dùng giả định hộp chữ nhật. Mô hình chạy lại góc lái và gia tốc đã đo; không dự báo bộ chấp hành lái.

Mô hình offline trong `mo_hinh_xe.py` nhận bộ tham số từ JSON, không có bộ tham số xe mặc định. Cần: `wheelbase_m`, `mass_kg`, `lateral.cf_over_mass`, `cr_over_mass`, `lf_fraction`, `iz_over_mass`, cùng các gain dọc `traction_gain`, `brake_gain`, `rolling_accel`, `linear_drag`. Low-speed transition mặc định 0.8 m/s; tích phân Runge–Kutta bậc 4, bước con tối đa 0.002 s. Tham số vật lý phải dương hữu hạn; các gain dọc không âm.

## 7. So sánh/đánh giá mô hình offline

- `so_sanh_mo_hinh.py fit`: fit từ các run train do người dùng chỉ định; JSON kết quả là nguồn thông số mô hình.
- `so_sanh_mo_hinh.py compare`: bước tích phân tối đa 0.001 s; horizon mặc định 0.1, 0.5, 1.0 s; stride 1.0 s.
- `doi_chieu_maneuver.py`: max step 0.001 s; tỷ lệ phân bổ phanh cầu trước và phanh tổng hợp phải truyền rõ; tùy chọn điều kiện theo gia tốc dọc.
- `danh_gia_phan_doc.py`: train/test và thư mục đầu ra phải truyền rõ; max step 0.001 s (giới hạn cho phép tối đa 0.01 s).

## 8. Công cụ kiểm tra

- `kiem_tra_moi_truong.py`: host/port 127.0.0.1:2000, timeout 15 s, Tesla Model 3; có `--offline` để không cần server.
- `kiem_tra_render.py`: timeout 90 s; tick kiểm tra renderer 0.05 s; thử thu 80 frame camera (các tick kiểm tra ban đầu 40 frame).
- `xac_nhan_server_gpu.py`: timeout RPC 90 s, chờ khởi động server 120 s; kiểm tra một bài J-turn 30 km/h và một lượt camera riêng. Thành công GPU cần log renderer gắn với tiến trình mới và camera render xác nhận; đây không phải xác nhận hiệu năng toàn bộ ma trận.
- `carla_runtime.py`: kiểm tra kết nối, đúng 4 bánh, khối lượng dương hữu hạn và góc lái bánh trước hợp lệ; khi dọn lượt, phanh rồi hủy xe, hoặc đóng băng nếu chế độ giữ xe được bật.

## 9. Cài đặt môi trường

- Hướng dẫn môi trường nhắm đến Ubuntu x86_64, CPython 3.10, glibc 2.27 trở lên và CARLA 0.9.15; dùng môi trường `.venv`.
- Các gói Python lấy theo `requirements.txt` (NumPy, pandas, SciPy, matplotlib và scikit-learn; phiên bản được khóa trong file đó).
- `run_ubuntu.sh` tạo môi trường/cài gói; `06_CHAY_QUAN_SAT_LINUX.sh` chạy quan sát xe có `role_name=hero`.

## 10. Nguồn và phạm vi

Giá trị mặc định được đối chiếu từ các script trong thư mục gốc, các tệp `DOC_TRUOC.txt`, `MO_HINH_VA_DON_VI.txt`, hướng dẫn maneuver và báo cáo kiểm tra. Thư mục `ket_qua_ngoai_tuyen_CHI_PHAN_DOC` được giữ nguyên; không sử dụng nó để suy ra giá trị mặc định chung của chương trình. Kết quả mô phỏng CARLA không phải phép đo xe thật, chứng nhận ISO, hay xác nhận rằng mô hình lốp đại diện chính xác cho xe vật lý.

## 11. Danh mục đầy đủ biến được ghi

### CSV bài cơ bản (`buoc1_carla.py`, `states.csv`)

| Nhóm/cột | Ý nghĩa | Đơn vị / ghi chú |
|---|---|---|
| `frame`, `t_s`, `t_sim_s` | Frame, thời gian từ đầu lượt, thời gian mô phỏng tuyệt đối | frame, s, s |
| `X_m`, `Y_m`, `Z_m` | Vị trí actor; Y đã đổi dấu theo quy ước thân xe | m |
| `vz_mps`, `vx_mps`, `vy_mps` | Vận tốc đứng và vận tốc dọc/ngang hệ thân xe | m/s |
| `psi_rad` | Hướng/yaw xe, dương trái | rad |
| `ax_mps2`, `ay_mps2` | Gia tốc dọc/ngang hệ thân xe | m/s² |
| `pitch_deg`, `roll_deg` | Góc chúc/nghiêng | độ |
| `angular_z_snapshot_raw`, `r_yaw_difference_radps` | Tốc độ góc Z CARLA và yaw rate sai phân từ góc hướng | rad/s |
| `steer_fl_deg_carla`, `steer_fr_deg_carla` | Góc bánh trước trái/phải từ CARLA | độ, quy ước CARLA |
| `delta_equiv_rad`, `delta_conversion` | Góc lái tương đương Ackermann và cách quy đổi | rad; không phải góc vô lăng |
| `throttle_applied`, `brake_applied`, `steer_applied_norm` | Lệnh đang tác dụng | chuẩn hóa |
| `throttle_next`, `brake_next`, `steer_next_norm`, `phase_next` | Lệnh bước kế tiếp và pha kịch bản | chuẩn hóa / chuỗi |

### CSV realtime (`mo_phong_thoi_gian_thuc.py`)

Các cột trạng thái gồm `frame`, `t_sim_s`, `X_m`, `Y_m`, `Z_m`, `psi_rad`, `roll_deg`, `pitch_deg`, `vx_mps`, `vy_mps`, `speed_kmh`, `r_radps`, `ax_mps2`, `ay_mps2`, `beta_rad`, `delta_equiv_rad`, `steer_fl_deg`, `steer_fr_deg`, `throttle`, `brake`, `steer_norm`, `gear`, `mass_kg`, `tire_friction_mean`, `Fx_total_est_N`, `Fy_total_est_N`.

`beta_rad` là sideslip thân xe từ `atan2(vy,vx)`. `Fx_total_est_N` và `Fy_total_est_N` chỉ là khối lượng nhân gia tốc, không phải lực lốp đo trực tiếp. Các góc/yaw đã quy đổi sang quy ước thân xe của chương trình.

Khi API telemetry bánh xe có mặt, mỗi bánh `FL`, `FR`, `RL`, `RR` có `tire_friction`, `lat_slip`, `long_slip`, `omega`, `tire_load`, `normalized_tire_load`, `torque`, `long_force`, `lat_force`; ví dụ `FL_lat_slip`. Ngoài ra có `Fy_front_tel_N`, `Fy_rear_tel_N`, `front_slip_tel`, `rear_slip_tel`. Không có API thì cột vẫn được tạo với `NaN`.

**Giới hạn phiên bản (đã đối chiếu API chính thức):** Python API được công bố cho CARLA 0.9.15 và 0.9.16 không liệt kê getter `get_telemetry_data()` hoặc getter trả tải/lực/slip từng bánh. 0.9.16 có `show_debug_telemetry()` để hiển thị telemetry trong spectator nhưng không trả dữ liệu đó về Python. Firmware chỉ dò method nếu server/client tùy biến có cung cấp; với bản stock, các trường bánh xe để `NaN`, còn góc trượt trước/sau tính từ trạng thái xe được đánh dấu là ước lượng mô hình.

### Mô hình bóng realtime

CSV có `model_active`, `model_age_s`, `model_X_m`, `model_Y_m`, `model_psi_rad`, `model_vx_mps`, `model_vy_mps`, `model_r_radps`, `err_vy_mps`, `err_r_radps`, `err_psi_rad`, `err_pos_m`. Sai số là mô hình trừ CARLA; mô hình mặc định tái khởi tạo mỗi giây nên đây không phải rollout dài không reset.

### Maneuver và metadata

`kiem_thu_maneuver.py` ghi vị trí, vận tốc/gia tốc thân xe, hướng/yaw, roll/pitch, góc bánh trước trái/phải và góc tương đương, điều khiển đang áp dụng/bước kế tiếp, pha, frame/thời gian. `metadata.json` lưu blueprint, khối lượng, chiều dài cơ sở và nguồn tính, ma sát yêu cầu/đọc lại/ban đầu từng bánh, phiên bản CARLA, cấu hình và quy ước tọa độ. Bài maneuver không ghi trực tiếp slip, lực hoặc tải lốp.

### Các góc trượt khác nhau

- `beta_rad`: sideslip toàn thân xe từ vận tốc dọc/ngang.
- `front_slip_rad`, `rear_slip_rad`: góc trượt tính trong mô hình tuyến tính từ trạng thái và góc bánh xe.
- `FL_lat_slip`, `FL_long_slip` (tương tự các bánh khác): telemetry CARLA, chỉ có nếu API hỗ trợ.
- Không có trường góc vô lăng; chương trình ghi góc bánh xe.

### JSON thông số và nhận dạng xe

`tham_so_carla.json` đọc từ CARLA khối lượng, drag coefficient, tâm khối lượng, max RPM, mô men quán tính, final ratio, tham số hộp số, torque/steering curve, thông số mỗi bánh (ma sát, damping, góc lái cực đại, bán kính, mô men phanh, độ cứng), vị trí bánh/bounding box, wheelbase và nguồn suy ra. Tệp còn lưu phiên bản client/server, map, mode, tình trạng telemetry và thông số ma sát.

`mo_hinh_truc_tuyen.json` lưu wheelbase, khối lượng, ngưỡng tốc độ thấp, gain dọc, `cf_over_mass`, `cr_over_mass`, `lf_fraction`, `iz_over_mass`, nguồn/số mẫu nhận dạng, trạng thái dữ liệu, tải lốp trung bình so với trọng lượng (nếu có telemetry), số lần reset/lỗi và sai số mô hình. Đây là tham số mô hình hiệu dụng, không nhất thiết là thông số cấu tạo vật lý.

## 12. Bản đồ tệp kết quả

| Tệp | Nội dung |
|---|---|
| `states.csv` | Trạng thái bài cơ bản hoặc maneuver |
| CSV realtime | Trạng thái CARLA, telemetry lốp nếu có, mô hình bóng |
| `metadata.json` | Thông số xe/bài, phiên bản, quy ước tọa độ |
| `tham_so_carla.json` | Vật lý xe đọc từ CARLA |
| `mo_hinh_truc_tuyen.json` | Tham số nhận dạng và thống kê mô hình bóng |
| `collision_events.json`, `run_status.json` | Va chạm và trạng thái lượt |
| `metrics.csv`, `validation_report.json` | Chỉ số so sánh mô hình offline |
| `camera/`, `render_report.json` | Frame/chẩn đoán render nếu chạy kiểm tra camera |

Dùng `--out` hoặc `run_status.json` để xác định thư mục kết quả cụ thể của lượt realtime.

