/**
 * KSB048 V0.010
 */
//% weight=10 color=#00A6F0 icon="\uf085" block="KSB048"
namespace KSB048 {
    
    const SERVOMIN = 104 // this is the 'minimum' pulse length count (out of 4096)
    const SERVOMAX = 510 // this is the 'maximum' pulse length count (out of 4096)
    const IIC_ADDRESS = 0x40
    const MODE1 = 0x00
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06

    
    export enum ServoNum {
        S0 = 0,
        S1 = 1,
        S2 = 2,
        S3 = 3,
        S4 = 4,
        S5 = 5,
        S6 = 6,
        S7 = 7,
        S8 = 8,
        S9 = 9,
        S10 = 10,
        S11 = 11,
     }
     export enum MotorNum {
        //% blockId="M1A" block="M1A"
        M1A = 0,
        //% blockId="M1B" block="M1B"
        M1B = 1,
        //% blockId="M2A" block="M2A"
        M2A = 2,
        //% blockId="M2B" block="M2B"
        M2B = 3,

    }

    export enum MecanumState {
        //% blockId="Go_Forward" block="Forward"
        Forward = 0,
        //% blockId="Car_Back" block="Backward"
        Back = 1,
        //% blockId="Go_Left" block="Left"
        Left = 2,
        //% blockId="GO_Right" block="Right"
        Right = 3,
        //% blockId="GO_Forward_Left" block="Forward_Left"
        Forward_Left = 4,
        //% blockId="GO_Forward_Right" block="Forward_Right"
        Forward_Right = 5,
        //% blockId="GO_Back_Left" block="Backward_Left"
        Back_Left = 6,
        //% blockId="GO_Back_Right" block="Backward_Right"
        Back_Right = 7,
        //% blockId="GO_Clockwise" block="Clockwise"
        Clockwise = 8,
        //% blockId="GO_Counterclockwise" block="Counterclockwise"
        Counterclockwise = 9,
        //% blockId="Go_Stop" block="Stop"
        Stop = 10,

    }


    export enum LedNum {
        //% blockId="Left_LED" block="Left"
        L_LED = 0,
        //% blockId="Right_LED" block="Right"
		R_LED = 1,

    }

    export enum FrqState {
        //% blockId="Frq_A" block="A"
        A = 0,
        //% blockId="Frq_B" block="B"
        B = 1,
        //% blockId="Frq_C" block="C"
        C = 2,
        //% blockId="Frq_D" block="D"
        D = 3,
        //% blockId="Frq_E" block="E"
        E = 4,
        //% blockId="Frq_F" block="F"
        F = 5,
     
    }

    

    let initialized = false;
    let neoStrip: neopixel.Strip;
	
    function i2c_write(reg: number, value: number) {
        
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(IIC_ADDRESS, buf)
    }

    function i2c_read(reg: number){
        
        pins.i2cWriteNumber(IIC_ADDRESS, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(IIC_ADDRESS, NumberFormat.UInt8BE);
        return val;
    }

    function init(): void {
        pins.setPull(DigitalPin.P8, PinPullMode.PullUp);
        pins.setPull(DigitalPin.P12, PinPullMode.PullUp);
        i2c_write(MODE1, 0x00);
        // Constrain the frequency
        setFreq(50);
        initialized = true;
    }

    function setFreq(freq: number): void {
        
        let prescaleval = 25000000/4096/freq;
        prescaleval -= 1;
        let prescale = prescaleval; 
        //let prescale = 121;
        let oldmode = i2c_read(MODE1);        
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2c_write(MODE1, newmode); // go to sleep
        i2c_write(PRESCALE, prescale); // set the prescaler
        i2c_write(MODE1, oldmode);
        control.waitMicros(5000);
        i2c_write(MODE1, oldmode | 0xa0);
	}
	
	function setPwm(channel: number, on: number, off: number): void {
		if (channel < 0 || channel > 15)
            return;

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on>>8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off>>8) & 0xff;
        pins.i2cWriteBuffer(IIC_ADDRESS, buf);
	}	

	function servo_map(x: number, in_min: number, in_max: number, out_min: number, out_max: number)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    function motor_map(x: number)
    {
        x = x*16; // map 255 to 4096
		if(x > 4095){
			x= 4095;
		}
		if(x < -4095){
			x= -4095;
		}
        return x;
    }
    

    

    //% blockId=KSB048_Ultrasonic 
    //% block="Ultrasonic(cm)"
    //% weight=98
    export function Ultrasonic(): number {

        let maxCmDistance = 500
        // send pulse
        pins.setPull(DigitalPin.P13, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P13, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P13, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P13, 0);

        const d = pins.pulseIn(DigitalPin.P13, PulseValue.High, maxCmDistance * 58);
        // read pulse
        
        return Math.idiv(d, 58);
    }

 
    //% blockId="KSB048_RGB" 
    //% block="RGB LED "
    //% weight=96
    export function RGB_LED(): neopixel.Strip {
        if (!neoStrip) {
            neoStrip = neopixel.create(DigitalPin.P16, 2, NeoPixelMode.RGB)
           
        }
 
        return neoStrip;
    }

    /**
     * Used to move the given servo to the specified degrees (0-180) connected to the KSB048
     * @param channel The number (1-16) of the servo to move
     * @param degrees The degrees (0-180) to move the servo to 
     */
    //% blockId=KSB048_Servo
    //% block="Servo channel %channel|degrees %degree"
    //% weight=86
    //% degree.min=0 degree.max=180
	export function Servo(channel: ServoNum, degree: number): void {
        
        if(!initialized){
			init()
		}
		// 50hz: 20,000 us
        //let servo_timing = (degree*1800/180+600) // 0.55 ~ 2.4
        //let pulselen = servo_timing*4096/20000
        //normal 0.5ms~2.4ms
        //SG90 0.5ms~2.0ms

        let pulselen = servo_map(degree, 0, 180, SERVOMIN, SERVOMAX);
        //let pulselen = servo_map(degree, 0, 180, servomin, servomax);
        setPwm(channel, 0, pulselen);
  
    }
    
	/**
     * Used to move the given servo to the specified degrees (0-180) connected to the KSB048
     * @param channel The number (1-16) of the servo to move
     * @param degrees The degrees (0-180) to move the servo to
     * @param servomin 'minimum' pulse length count ; eg: 112
     * @param servomax 'maximum' pulse length count ; eg: 491
     */
    //% blockId=KSB048_ServoRange
    //% block="Servo channel %channel|degrees %degree|servomin %servomin|servomax %servomax"
    //% degree.min=0 degree.max=180
	export function ServoRange(channel: ServoNum, degree: number, servomin: number, servomax: number): void {
        
        if(!initialized){
			init()
		}
		// 50hz: 20,000 us
        //normal 0.5ms~2.4ms
        //SG90 0.5ms~2.0ms
        // servomin Servo_min_timing (ms)*1000*4096/20000 
        // servomax Servo_max_timing (ms)*1000*4096/20000 
        // let pulselen = servo_map(degree, 0, 180, SERVOMIN, SERVOMAX);
        let pulselen = servo_map(degree, 0, 180, servomin, servomax);
        setPwm(channel, 0, pulselen);

    }

    //% blockId=KSB048_Motor 
    //% block="Motor channel %channel|speed %speed"
	//% weight=85
	//% speed.min=-255 speed.max=255
    export function Motor(channel: MotorNum, speed: number): void {
		if(!initialized){
			init()
        }
        let pwm1 =0;
        let pwm2 =0;
        speed=motor_map(speed);

        switch(channel){

            case 0:{
                pwm1 = 11;
                pwm2 = 10;                
                break;
            }
            case 1:{
                pwm1 = 8;
                pwm2 = 9;                
                break;
            }
            case 2:{
                pwm1 = 12;
                pwm2 = 13;               
                break;
            }
            case 3:{
                pwm1 = 15;
                pwm2 = 14;              
                break;
            }
            
        }
        

		if(speed>=0){
			setPwm(pwm1, 0, speed)
			setPwm(pwm2, 0, 0)
		}else{
			setPwm(pwm1, 0, 0)
			setPwm(pwm2, 0, -speed)
        }
            
    }

    //% blockId=KSB048_Mecanum_Car
    //% block="Mecanum_Car %index|Speed %speed"
    //% weight=87
    //% speed.min=0 speed.max=255
    export function Mecanum_Car(index: MecanumState, speed: number): void {
        switch (index) {
            case MecanumState.Forward:
                Motor(MotorNum.M1B, speed);
                Motor(MotorNum.M1A, speed);
                Motor(MotorNum.M2B, speed);
                Motor(MotorNum.M2A, speed);
                break;
            case MecanumState.Back:
                Motor(MotorNum.M1B, -speed);
                Motor(MotorNum.M1A, -speed);
                Motor(MotorNum.M2B, -speed);
                Motor(MotorNum.M2A, -speed);
                break;
            case MecanumState.Left:
                Motor(MotorNum.M1B, -speed);
                Motor(MotorNum.M1A, speed);
                Motor(MotorNum.M2B, speed);
                Motor(MotorNum.M2A, -speed);
                break;
            case MecanumState.Right:
                Motor(MotorNum.M1B, speed);
                Motor(MotorNum.M1A, -speed);
                Motor(MotorNum.M2B, -speed);
                Motor(MotorNum.M2A, speed);
                break;
            case MecanumState.Forward_Left:
                Motor(MotorNum.M1B, 0);
                Motor(MotorNum.M1A, speed);
                Motor(MotorNum.M2B, speed);
                Motor(MotorNum.M2A, 0);
                break;
            case MecanumState.Forward_Right:
                Motor(MotorNum.M1B, speed);
                Motor(MotorNum.M1A, 0);
                Motor(MotorNum.M2B, 0);
                Motor(MotorNum.M2A, speed);
                break;
            case MecanumState.Back_Left:
                Motor(MotorNum.M1B, -speed);
                Motor(MotorNum.M1A, 0);
                Motor(MotorNum.M2B, 0);
                Motor(MotorNum.M2A, -speed);
                break;
            case MecanumState.Back_Right:
                Motor(MotorNum.M1B, 0);
                Motor(MotorNum.M1A, -speed);
                Motor(MotorNum.M2B, -speed);
                Motor(MotorNum.M2A, 0);
                break;
            case MecanumState.Clockwise:
                Motor(MotorNum.M1B, speed);
                Motor(MotorNum.M1A, -speed);
                Motor(MotorNum.M2B, speed);
                Motor(MotorNum.M2A, -speed);
                break;
            case MecanumState.Counterclockwise:
                Motor(MotorNum.M1B, -speed);
                Motor(MotorNum.M1A, speed);
                Motor(MotorNum.M2B, -speed);
                Motor(MotorNum.M2A, speed);
                break;
            case MecanumState.Stop:
                Motor(MotorNum.M1B, 0);
                Motor(MotorNum.M1A, 0);
                Motor(MotorNum.M2B, 0);
                Motor(MotorNum.M2A, 0);
                break;

        }
    }
 


}
