/// Blocks for a specified number of milliseconds
void wait_ms(unsigned int time_val);

void ADC_init();

unsigned int read_ADC();

int read_IR_distance();

void ping_timer_init();

void send_pulse();

float read_PING_distance();

void servo_timer_init();

void move_servo(volatile float* degrees);

void USART_Init(unsigned int ubrr);

void USART_Transmit(unsigned char data);

unsigned char USART_Receive(void);

void send_message(char *message);

