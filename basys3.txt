`timescale 1ns / 1ps

// ============================================================================
// 1. 최상위 모듈 (dog_top)
//    - 라즈베리파이와 연결되어 모든 장치를 총괄합니다.
// ============================================================================
module dog_top(
    input clk,              // 100MHz 시스템 클럭
    input reset_p,          // 중앙 버튼 (전체 리셋)
    input sw,               // 스위치 (모터 안전장치: 올리면 동작)
    
    // [통신 1] 모터 제어용 (115200bps, GPIO 14 -> JC1)
    input rx_motor,      
    
    // [통신 2] LCD 이미지용 (460800bps, GPIO 8 -> JA9/H2)
    input rx_lcd,        

    // [출력 1] 모터 & 서보 & LED
    output [3:0] motor_in, // L298N 방향제어
    output [1:0] motor_en, // L298N 속도제어(PWM)
    output sg90_pwm,       // 목 회전 서보모터
    output [15:0] led,     // 상태 확인용 LED

    // [출력 2] LCD 디스플레이 (Pmod JA)
    output lcd_reset,
    output lcd_cs,
    output lcd_dc,
    output lcd_sclk,
    output lcd_sdi,
    output lcd_bl
    );

    // --------------------------------------------------------
    // (1) 모터/서보 패킷 처리부
    // --------------------------------------------------------
    wire [7:0] motor_rx_data;
    wire motor_rx_done;
    wire [14:0] w_high_dur;

    reg [7:0] p_angle;
    reg [7:0] p_l_speed;
    reg [7:0] p_r_speed;
    reg [3:0] p_dir;
    
    reg [2:0] parse_state;

    // 모터용 UART 수신 (115200bps)
    uart_rx_motor motor_rx_inst (
        .clk(clk), .reset_p(reset_p), .rx_in(rx_motor), 
        .data(motor_rx_data), .done(motor_rx_done)
    );

    // 패킷 해석: [0xFF, Angle, L_Speed, R_Speed, Dir]
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            parse_state <= 0;
            p_angle <= 90; p_l_speed <= 0; p_r_speed <= 0; p_dir <= 0;
        end else if (motor_rx_done) begin
            case (parse_state)
                0: if (motor_rx_data == 8'hFF) parse_state <= 1; // 헤더 확인
                1: begin p_angle <= motor_rx_data; parse_state <= 2; end
                2: begin p_l_speed <= motor_rx_data; parse_state <= 3; end
                3: begin p_r_speed <= motor_rx_data; parse_state <= 4; end
                4: begin p_dir <= motor_rx_data; parse_state <= 0; end
            endcase
        end
    end

    // --------------------------------------------------------
    // (2) 하드웨어 구동부 연결
    // --------------------------------------------------------
    
    // 서보 모터 (각도 -> PWM 변환)
    mg996r_cntr servo_calc (.angle_in(p_angle), .high_dur(w_high_dur));
    pwm_gen servo_pwm (.clk(clk), .reset_p(reset_p), .high_dur(w_high_dur), .pwm(sg90_pwm));

    // DC 모터 (L298N 드라이버)
    robot_leg_motor_ctl motor_driver(
        .clk(clk), .reset_p(reset_p), 
        .in_left_speed(p_l_speed), 
        .in_right_speed(p_r_speed), 
        .in_direction(p_dir),
        .motor_in(motor_in), .motor_en(motor_en), 
        .led(led), .sw(sw)
    );

    // LCD 디스플레이 (ILI9341 SPI)
    ili9341_spi_robot_face lcd_inst (
        .clk(clk),
        .btnC(reset_p),     // 리셋 공유
        .rx(rx_lcd),        // LCD용 고속 UART (460800bps)
        .lcd_reset(lcd_reset),
        .lcd_cs(lcd_cs),
        .lcd_dc(lcd_dc),
        .lcd_sclk(lcd_sclk),
        .lcd_sdi(lcd_sdi),
        .lcd_bl(lcd_bl)
    );

endmodule


// ============================================================================
// 2. LCD 제어 모듈 (ILI9341) - ★ 업데이트된 버전 ★
//    - 16비트 픽셀 데이터 조립 기능이 포함되어 색상 깨짐이 없습니다.
// ============================================================================
module ili9341_spi_robot_face (
    input  wire clk,          // 100MHz
    input  wire btnC,         // Reset
    input  wire rx,           // UART RX (460,800 baud)
    output reg  lcd_reset,    
    output reg  lcd_cs,       
    output reg  lcd_dc,       
    output reg  lcd_sclk,     
    output wire lcd_sdi,      
    output wire lcd_bl        
);

    assign lcd_bl = 1'b1; // 백라이트 켜기

    localparam S_RESET      = 4'd0, S_DELAY       = 4'd1,
               S_INIT       = 4'd2, S_SET_ADDR    = 4'd3,
               S_WAIT_BYTE  = 4'd4, S_SEND_SPI    = 4'd5;

    reg [3:0]  state = S_RESET;
    reg [3:0]  after_delay_state;
    reg [31:0] delay_cnt = 0;
    reg [31:0] idle_cnt = 0; 
    reg [7:0]  send_data = 0;
    reg [7:0]  cmd_step = 0;
    reg [3:0]  bit_cnt = 0;
    
    // [핵심] 16비트 픽셀 조립용 레지스터
    reg [15:0] pixel_reg = 0;
    reg [4:0]  spi_div = 0; 
    reg [16:0] pixel_cnt = 0;
    reg        pixel_byte_toggle = 0; // 0:상위바이트, 1:하위바이트

    assign lcd_sdi = send_data[7 - bit_cnt];

    wire [7:0] rx_data;
    wire       rx_ready;
    
    // 고속 UART (LCD 전용)
    uart_rx_high_speed #(.BAUD(460800)) uart_inst (.clk(clk), .rx(rx), .data(rx_data), .ready(rx_ready));

    always @(posedge clk) begin
        if (btnC) begin
            state <= S_RESET; 
            lcd_reset <= 0; lcd_cs <= 1; lcd_sclk <= 0;
            delay_cnt <= 0; pixel_cnt <= 0; pixel_byte_toggle <= 0;
            cmd_step <= 0; idle_cnt <= 0;
        end else begin
            case (state)
                S_RESET: begin 
                    lcd_reset <= 0;
                    if (delay_cnt >= 5_000_000) begin 
                        lcd_reset <= 1; 
                        delay_cnt <= 0; 
                        state <= S_DELAY; 
                        after_delay_state <= S_INIT;
                    end else delay_cnt <= delay_cnt + 1;
                end

                S_DELAY: begin 
                    if (delay_cnt >= 20_000_000) begin 
                        delay_cnt <= 0; 
                        state <= after_delay_state; 
                    end else delay_cnt <= delay_cnt + 1;
                end

                S_INIT: begin 
                    lcd_dc <= 0;
                    case (cmd_step)
                        0: begin send_data <= 8'h01; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        1: begin send_data <= 8'h11; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        2: begin send_data <= 8'h36; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        3: begin lcd_dc <= 1; send_data <= 8'h08; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        4: begin lcd_dc <= 0; send_data <= 8'h3A; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        5: begin lcd_dc <= 1; send_data <= 8'h55; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        6: begin lcd_dc <= 0; send_data <= 8'h29; state <= S_SEND_SPI; after_delay_state <= S_INIT; end 
                        default: begin state <= S_SET_ADDR; cmd_step <= 0; end
                    endcase
                    cmd_step <= cmd_step + 1;
                end

                S_SET_ADDR: begin 
                    case (cmd_step)
                        0: begin lcd_dc <= 0; send_data <= 8'h2A; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        1,2,3: begin lcd_dc <= 1; send_data <= 8'h00; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end 
                        4: begin lcd_dc <= 1; send_data <= 8'hEF; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        5: begin lcd_dc <= 0; send_data <= 8'h2B; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        6,7: begin lcd_dc <= 1; send_data <= 8'h00; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        8: begin lcd_dc <= 1; send_data <= 8'h01; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        9: begin lcd_dc <= 1; send_data <= 8'h3F; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        10:begin lcd_dc <= 0; send_data <= 8'h2C; state <= S_SEND_SPI; after_delay_state <= S_SET_ADDR; end
                        default: begin state <= S_WAIT_BYTE; pixel_cnt <= 0; idle_cnt <= 0; pixel_byte_toggle <= 0; end
                    endcase
                    cmd_step <= cmd_step + 1;
                end

                S_WAIT_BYTE: begin 
                    if (idle_cnt > 5_000_000) begin // 50ms 타임아웃
                         idle_cnt <= 0;
                         pixel_cnt <= 0;
                         pixel_byte_toggle <= 0;
                         state <= S_SET_ADDR; cmd_step <= 0;
                    end else if (rx_ready) begin
                        idle_cnt <= 0;
                        if (pixel_byte_toggle == 0) begin 
                            pixel_reg[15:8] <= rx_data; 
                            pixel_byte_toggle <= 1; 
                        end else begin
                            pixel_reg[7:0] <= rx_data; 
                            pixel_byte_toggle <= 0;
                            send_data <= pixel_reg[15:8]; // 상위 바이트 먼저 전송 준비
                            lcd_dc <= 1; 
                            state <= S_SEND_SPI; 
                            after_delay_state <= S_SEND_SPI; 
                        end
                    end else idle_cnt <= idle_cnt + 1;
                end

                S_SEND_SPI: begin 
                    lcd_cs <= 0;
                    if (spi_div >= 6) begin 
                        spi_div <= 0;
                        if (lcd_sclk == 0) lcd_sclk <= 1;
                        else begin
                            lcd_sclk <= 0;
                            if (bit_cnt == 7) begin
                                bit_cnt <= 0; lcd_cs <= 1;
                                // 픽셀 데이터 전송 중일 때 (상위 -> 하위 순서 처리)
                                if (after_delay_state == S_SEND_SPI) begin 
                                    send_data <= pixel_reg[7:0]; // 하위 바이트 장전
                                    after_delay_state <= S_WAIT_BYTE; // 이거 보내고 다시 수신 대기
                                    state <= S_SEND_SPI; 
                                end else begin
                                    // 일반 명령이거나 픽셀 하위바이트까지 다 보냈을 때
                                    if (after_delay_state == S_WAIT_BYTE) begin
                                        if (pixel_cnt >= 76799) begin 
                                            state <= S_SET_ADDR; cmd_step <= 0; 
                                        end else begin 
                                            pixel_cnt <= pixel_cnt + 1; 
                                            state <= S_WAIT_BYTE; 
                                        end
                                    end else state <= (after_delay_state == S_INIT) ? S_DELAY : after_delay_state;
                                end
                            end else bit_cnt <= bit_cnt + 1;
                        end
                    end else spi_div <= spi_div + 1;
                end
            endcase
        end
    end
endmodule


// ============================================================================
// 3. DC 모터 제어 모듈
// ============================================================================
module robot_leg_motor_ctl(
    input clk, reset_p,
    input [7:0] in_left_speed,
    input [7:0] in_right_speed,
    input [3:0] in_direction,
    output [3:0] motor_in,
    output [1:0] motor_en,
    output reg [15:0] led,
    input sw
    );

    parameter PERIOD = 200000; // 500Hz
    reg [19:0] counter;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) counter <= 0;
        else begin
            if (counter < PERIOD - 1) counter <= counter + 1;
            else counter <= 0;
        end
    end

    wire [19:0] duty_left  = in_left_speed * 2000;
    wire [19:0] duty_right = in_right_speed * 2000;

    assign motor_in = (sw) ? in_direction : 4'b0000;
    assign motor_en[0] = (sw) ? (counter < duty_left) : 1'b0;
    assign motor_en[1] = (sw) ? (counter < duty_right) : 1'b0;

    always @(*) begin
        led = 0;
        if (sw) begin
            if (in_left_speed > 0 || in_right_speed > 0) led[0] = 1;
            else led[1] = 1;
            led[5:2] = in_direction; 
        end
    end
endmodule


// ============================================================================
// 4. UART 수신 모듈 (모터용: 115200bps)
// ============================================================================
module uart_rx_motor (
    input clk, reset_p, rx_in,
    output reg [7:0] data, 
    output reg done
);
    parameter CLK_FREQ = 100_000_000;
    parameter BAUD_RATE = 115200;
    parameter TICKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg [15:0] tick_cnt; 
    reg [3:0] bit_cnt;
    reg rx_reg1, rx_reg2, active; 
    reg [7:0] sh_reg;

    always @(posedge clk) begin
        rx_reg1 <= rx_in; rx_reg2 <= rx_reg1;
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            tick_cnt <= 0; bit_cnt <= 0; active <= 0; done <= 0; data <= 0; sh_reg <= 0;
        end else begin
            done <= 0;
            if (!active) begin
                if (rx_reg2 == 0) begin 
                    active <= 1; tick_cnt <= 0; bit_cnt <= 0;
                end
            end else begin
                if (tick_cnt < TICKS_PER_BIT - 1) tick_cnt <= tick_cnt + 1;
                else begin
                    tick_cnt <= 0;
                    if (bit_cnt == 0) begin 
                        tick_cnt <= TICKS_PER_BIT / 2; bit_cnt <= 1;
                    end else if (bit_cnt <= 8) begin
                        sh_reg[bit_cnt-1] <= rx_reg2; bit_cnt <= bit_cnt + 1;
                    end else begin
                        active <= 0; bit_cnt <= 0; done <= 1; data <= sh_reg;
                    end
                end
            end
        end
    end
endmodule


// ============================================================================
// 5. UART 수신 모듈 (LCD용: 고속 460800bps)
// ============================================================================
module uart_rx_high_speed #(parameter BAUD = 460800, parameter CLK_FREQ = 100000000) (
    input wire clk,
    input wire rx,
    output reg [7:0] data,
    output reg ready
);
    localparam integer BIT_PERIOD = CLK_FREQ / BAUD;
    localparam integer HALF_PERIOD = BIT_PERIOD / 2;

    reg [1:0] state = 0;
    reg [15:0] clk_cnt = 0;
    reg [2:0] bit_idx = 0;
    reg [7:0] shft = 0;
    reg r1, r2;

    always @(posedge clk) begin r1 <= rx; r2 <= r1; end

    always @(posedge clk) begin
        ready <= 0;
        case (state)
            0: if (r2 == 0) begin clk_cnt <= 0; state <= 1; end 
            1: if (clk_cnt >= HALF_PERIOD) begin clk_cnt <= 0; state <= 2; end else clk_cnt <= clk_cnt + 1;
            2: if (clk_cnt >= BIT_PERIOD-1) begin 
                   clk_cnt <= 0; shft[bit_idx] <= r2;
                   if (bit_idx == 7) begin bit_idx <= 0; state <= 3; end else bit_idx <= bit_idx + 1;
               end else clk_cnt <= clk_cnt + 1;
            3: if (clk_cnt >= BIT_PERIOD-1) begin data <= shft; ready <= 1; state <= 0; end else clk_cnt <= clk_cnt + 1;
        endcase
    end
endmodule


// ============================================================================
// 6. 서보 모터 제어 모듈
// ============================================================================
module mg996r_cntr(
    input [7:0] angle_in,     // 0~180
    output [14:0] high_dur    // PWM High Time (us)
    );
    wire [7:0] safe_angle = (angle_in > 180) ? 8'd180 : angle_in;
    wire [14:0] extended_angle = {7'd0, safe_angle};
    assign high_dur = 15'd500 + (extended_angle * 15'd11);
endmodule

module pwm_gen(
    input clk, reset_p,
    input [14:0] high_dur,
    output reg pwm
    );
    parameter CLK_FREQ = 100; // 1us Tick
    reg [8:0] cnt_1us;
    wire clk_1us = (cnt_1us == CLK_FREQ - 1);
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_1us <= 0;
        else if(clk_1us) cnt_1us <= 0;
        else cnt_1us <= cnt_1us + 1;
    end

    reg [14:0] cnt_20ms;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_20ms <= 0;
        else if(clk_1us) begin
            if(cnt_20ms >= 19999) cnt_20ms <= 0;
            else cnt_20ms <= cnt_20ms + 1;
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if(reset_p) pwm <= 0;
        else pwm <= (cnt_20ms < high_dur);
    end
endmodule
