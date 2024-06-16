//Copy to main.c to enable printf as private function prototype

int __io_putchar(int ch)
{
	//Write character to ITM ch.0
	ITM_SendChar(ch);
	return(ch);
}
