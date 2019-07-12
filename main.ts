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
        //% blockId="M1A" block="Right"
        M1A = 0,
        //% blockId="M1B" block="Left"
		M1B = 1,

    }
    export enum LedNum {
        //% blockId="Left_LED" block="Left"
        L_LED = 0,
        //% blockId="Right_LED" block="Right"
		R_LED = 1,

    }
    

    export enum RunState {
        //% blockId="Go_Forward" block="Forward"
        Forward = 1,
        //% blockId="Car_Back" block="Back"
        Back = 2,
        //% blockId="Go_Left" block="Left"
        Left = 3,
        //% blockId="GO_Right" block="Right"
        Right = 4,
        //% blockId="Go_Stop" block="Stop"
        Stop = 5,
     
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
        freq=freq*0.95;
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
        
        speed=motor_map(speed);
        
		let pwm1 = (channel*2)+12;
		let pwm2 = (channel*2)+13;
		if(speed>=0){
			setPwm(pwm1, 0, speed)
			setPwm(pwm2, 0, 0)
		}else{
			setPwm(pwm1, 0, 0)
			setPwm(pwm2, 0, -speed)
        }
            
    }
	
	


}
