#include <math.h>
#include <Encoder.h>

const double GRAVITY = 9.81;              // Gravitational acceleration (m/s^2)
const double CART_M = 0.2182;             // Mass of the cart (kg)
const double PENDULUM_M = 0.0756;         // Mass of the pendulum bob (kg)
const double LENGTH = 0.392 * 0.5;              // Length of the pendulum (m)
const double PULSE_TO_METER = 0.00001578116;

const double balancing_offset_radian = 15.0 * (PI / 180); 

const double balancing_offset = 0.22;     

bool moving_right = true;
unsigned long prev_millis_send_data = 0;

#define CLK1 2 
#define DT1 3

#define CLK2 18
#define DT2 19

Encoder encoderCart(CLK1, DT1);
Encoder encoderPendulum(CLK2, DT2);

// Define the pins connected to the BTS7960 motor driver
#define RPWM_PIN 9
#define LPWM_PIN 10
#define R_EN_PIN 8
#define L_EN_PIN 7

#define INTERVAL 75

// Previous values
double prev_millis = 0.0; 

double prev_theta = 0.0;

double prev_x = 0.0;

double current_velocity = 0.0;

bool is_balancing = false;

bool position_off = false;

double desired_position = 0.0;

double input_desired_position = 0.0;

void setup() {
  Serial.begin(9600);

  encoderPendulum.write(-10000);

  prev_theta = get_theta();

  // Set up the pins as output
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void loop() {
  //handles input.
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input_desired_position = input.toDouble() / 100.0;
    changeDesiredPosition(input_desired_position);
  }

  if(get_theta() > -balancing_offset_radian && get_theta() < balancing_offset_radian)
    is_balancing = true;

  else{
    is_balancing = false;
    init_prev();
  }
  
  if(abs(getCartPosition()) > balancing_offset){
    Serial.print("balancing set off. Position: " ); 
    Serial.println(getCartPosition());
    position_off = true;
    analogWrite(RPWM_PIN, 0);  // Full speed
    analogWrite(LPWM_PIN, 0);
  }

  if(!is_balancing || position_off){
    analogWrite(RPWM_PIN, 0);  // Full speed
    analogWrite(LPWM_PIN, 0);
    position_off = false;
    return;
  }
  //check whether position beyond offset to set is_balancing false.
  
  double x = getCartPosition();
  double theta = get_theta();

  // Geçen zaman (türevlerde kullanmak için)
  double cur_millis = millis();
  if (cur_millis - prev_millis < INTERVAL ){
      Serial.print(x, 8);  // Send value1 with 8 decimal places
      Serial.print(",");
      Serial.println(theta, 8);  // Send value2 with 8 decimal places and a newline

    return;
  }

  double delta_time = (cur_millis - prev_millis) * 0.001;

  //calculations
  double theta_d = (theta - prev_theta) / delta_time;
  double x_d = (x - prev_x) / delta_time;


  const double ke = 0.01986253689;

  double angularV = ((((x_d/PULSE_TO_METER) / 500.0) * 360.0) * PI/180);

  double eb = angularV * ke;

  double force = lqr(x - desired_position, x_d, theta, theta_d);

  const double kt = 0.0199;

  double I = force / kt;

  const double R = 0.161;


  double V = eb + I * R;

  setMotorPwm(V * 21.3333);

  // Assign previous values
  prev_millis = cur_millis;
  prev_theta = theta;

  prev_x = x; 

}

double get_theta(){
  long pendulumPosition = encoderPendulum.read();
  return ((pendulumPosition / 20000.0) * 360) * (PI / 180.0);
}

double log(double acceleration, double cur_millis, double theta, double x, double delta_time, double theta_d, double x_d, double rpm){
  Serial.print("cur_millis: "); Serial.println(cur_millis);
  Serial.print("theta: "); Serial.println(theta);
  Serial.print("x: "); Serial.println(x);
  Serial.print("delta_time : "); Serial.println(delta_time);
  Serial.print("theta_d: "); Serial.println(theta_d);
  Serial.print("x_d: "); Serial.println(x_d);
  Serial.print("prev_millis: "); Serial.println(prev_millis);
  Serial.print("prev_theta: "); Serial.println(prev_theta);
  Serial.print("prev_x: "); Serial.println(prev_x);
}

void init_prev(){
  // Previous values
	prev_millis = 0.0; 

	prev_theta = 0.0;

	prev_x = 0.0;

	current_velocity = 0.0;

	is_balancing = false;
}

double lqr(double x, double x_d, double theta, double theta_d){

  double k1 = -1.0;
  double k2 = -1.0593;
  double k3 = 6.1515;
  double k4 = 0.8486;

  double t1 = x * (-k1);
  double t2 = x_d * (-k2);
  double t3 = theta * (-k3);
  double t4 = theta_d * (-k4);

  return t1 + t2 + t3 + t4;
}

void setMotorPwm(float pwm){
  
  unsigned int pwm_to_set = 0;
  
  if(pwm < 0){
    pwm_to_set = -pwm + 0.70;
    moving_right = true;
  }
  else{
    pwm_to_set = pwm + 0.70;
    moving_right = false;
  }
  
  if(pwm_to_set > 60)
    pwm_to_set = 60;

  if(moving_right){
    analogWrite(RPWM_PIN, pwm_to_set);
    analogWrite(LPWM_PIN, 0);
  }
  else{
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwm_to_set);
  }

  delay(1);
}

double getCartPosition(){
  return encoderCart.read() * PULSE_TO_METER;
}

void changeDesiredPosition(float meters){
  if(meters > 0.2){
    desired_position = 0.2;
    Serial.println("Desired position can't be more than 20cm.\n");
  }
  else if(meters < -0.2){
    desired_position = -0.2;
    Serial.println("Desired position can't be more than 20cm.\n");
  }
  else{
    desired_position = meters;
  }
}
