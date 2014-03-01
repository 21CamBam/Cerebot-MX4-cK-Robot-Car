.GLOBAL main

.DATA                        # The start of the data segment

enable_display: .BYTE 27, '[', '3', 'e', '\0'
set_cursor: .BYTE 27, '[', '1', 'c', '\0'
home_cursor: .BYTE 27, '[', 'j', '\0'
wrap_line: .BYTE 27, '[', '0', 'h', '\0'

.TEXT                        # The start of the code segment

.ENT main                    # Setup a main entry point
main:
	JAL setupPORTs	# Setup pins to H-bridge

	# NOTE: H-bridge connected to Cerebot MX4cK Port JH-10:07
	# Set motor direction;
	# Be careful not to change the direction at the same time the motor is pulsed; may cause short cirucit in H-bridge
	LI $t0, 1 << 7
	SW $t0, LATDCLR

    LI $t0, 1 << 6
	SW $t0, LATDSET

    JAL setupMultiVectoredMode	# Want mult-vectored interrupts - each interrupt may have a location in interrupt vector table
	JAL setupOutputCompare2Module # Configure the duty cycle of PWM signal on OC1
    JAL setupOutputCompare3Module # Configure the duty cycle of PWM signal on OC2
	JAL setupTimer2	# Configure Timer 2 - Only Timers 2 and 3 should be used for compare time base
    JAL setupTimer3	# Configure Timer 2 - Only Timers 2 and 3 should be used for compare time base

	loop:
		# Event loop
       # JAL check_sensors

		J loop      # Embedded programs require that they run forever! So jump back to the beginning of the loop

.END main

.ENT setupPORTs
setupPORTs:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

    # Left
	# H-bridge DIR - JH-07, RD07; EN - JH-08, RD02, OC1 - Yes, the enable is controlled by output channel 3
	# Set these two pins to outputs
	LI $t0, 1 << 7
	SW $t0, TRISDCLR
	LI $t0, 1 << 1
	SW $t0, TRISDCLR

    # Right
    # H-bridge DIR - JH-07, RD06; EN - JH-08, RD01, OC1 - Yes, the enable is controlled by output channel 2
	# Set these two pins to outputs
	LI $t0, 1 << 6
	SW $t0, TRISDCLR
	LI $t0, 1 << 2
	SW $t0, TRISDCLR

    # PmodLS1
    # Set sensor pins to input
    LI $t0, 0xFFF0
    SW $t0, TRISB

    # PMOD CLS TX JE02 RF08, RX JE03 RF02..
    LA $s0, TRISF
	LW $t0, ($s0)
    ORI $t0, $t0, 1 << 2 # RF02
	ANDI $t0, $t0, 0b1111111011111111 #RF08
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupPORTs

.ENT setupMultiVectoredMode
setupMultiVectoredMode:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Interrupt control register
	LA $s0, INTCON # Register necessary for setting multi-vectored mode
	LW $t0, ($s0)
	ORI $t0, $t0, 1 << 12 # Set for mutli-vectored mode
	#SW $t0, ($s0)
	SW $t0, INTCON

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupMultiVectoredMode

.ENT setupOutputCompare3Module
setupOutputCompare3Module:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Ensure OC3 is off while setting up module 3
	LA $s0, OC3CON # Output compare 3 control register
	MOVE $t0, $zero
	SW $t0, ($s0)

	# Initialize the OC3R register before the output compare module, this register determins duty cycle
	LA $s0, OC3R
	LI $t0, 0x0180 # Shoot for 50% duty cycle, PR3 - 0x0200
	SW $t0, ($s0)
	# The OC3RS secondary output compare register will contain the actual duty cycle
	LA $s0, OC3RS
	LI $t0, 0x0180 # Shoot for 50% duty cycle
	SW $t0, ($s0)

	# Now configure the compare module using OC3CON
	# Bits 2:0 - 110 = PWM mode on OC3, 011 = compare event toggles OC3 pin
	# Bit 3 - 1 = Timer 3 clk src, 0 = Timer 2 clk src
	# Bit 5 - 1 = 32-bit comparisons, 0 = 16-bit comparisons
	# Bit 15 - 1 = enable output compare, 0 = disabled, not drawing current
	LA $s0, OC3CON
	MOVE $t0, $zero
    ORI $t0, 1 << 3
	ORI $t0, $t0, 6 # PWM mode
	ORI $t0, $t0, 1 << 15 # Enable output compare module
	SW $t0, ($s0)

	# Set priority of compare match interrupt IPC3<20:18>
	LA $s0, IPC3SET
	LI $t0, 14 # priority 14
	SLL $t0, $t0, 18
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupOutputCompare3Module

# Used for the clock source for the compare time base
.ENT setupTimer2
setupTimer2:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# T2CON - Control Register for Timer 2
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled
	LA $s0, T2CON
	LI $t0, 0x0 # stop Timer 2
	SW $t0, ($s0)

	# TMR2 register contains 16-bit current value of Timer 2
	LA $s0, TMR2
	MOVE $t0, $zero # clear timer value
	SW $t0, ($s0)

	# PR2 register contains 16-bit period match value, i.e. TMR2 value == PR2 value ==> timer resets
	LA $s0, PR2
	LI $t0, 0x01AA # Affects how often the timer is reset! Contains PWM period! Allows ~50% duty cycle based on OC2R values
	SW $t0, ($s0)

	# T2CON - Control Register for Timer 2 - can combine with Timer 3 to form 32-bit timer!
	# Bit 1 - TCS Timer Clock Source, 0 = internal peripheral clk (PBCLK)
	# Bit 3 - 1 = 32-bit timer, 0 = 16-bit timer
	# Bits 6:4 - TCKPS Timer Clock Prescale Select bits, these are slightly different than with Timer 1; 100 = /16
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled

	LA $s0, T2CON
	LI $t0, 0x8050 # PBCLK / 16, Timer 2 on, 16-bit timer mode, use PBCLK
	SW $t0, ($s0)

	# Will not set an interrupt priority for Timer 2, but will for Ouput Compare Module 2

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupTimer2

.ENT setupTimer3
setupTimer3:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# T3CON - Control Register for Timer 3
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled
	LA $s0, T3CON
	LI $t0, 0x0 # stop Timer 3
	SW $t0, ($s0)

	# TMR2 register contains 16-bit current value of Timer 3
	LA $s0, TMR3
	MOVE $t0, $zero # clear timer value
	SW $t0, ($s0)

	# PR2 register contains 16-bit period match value, i.e. TMR3 value == PR3 value ==> timer resets
	LA $s0, PR3
	LI $t0, 0x01AA # Affects how often the timer is reset! Contains PWM period! Allows ~50% duty cycle based on OC3R values
	SW $t0, ($s0)

	# T3CON - Control Register for Timer 3 - can combine with Timer 2 to form 32-bit timer!
	# Bit 1 - TCS Timer Clock Source, 0 = internal peripheral clk (PBCLK)
	# Bit 3 - 1 = 32-bit timer, 0 = 16-bit timer
	# Bits 6:4 - TCKPS Timer Clock Prescale Select bits, these are slightly different than with Timer 1; 100 = /16
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled

	LA $s0, T3CON
	LI $t0, 0x8050 # PBCLK / 16, Timer 3 on, 16-bit timer mode, use PBCLK
	SW $t0, ($s0)

	# Will not set an interrupt priority for Timer 3, but will for Ouput Compare Module 3

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupTimer3

.ENT setupOutputCompare2Module
setupOutputCompare2Module:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Ensure OC1 is off while setting up module 2
	LA $s0, OC2CON # Output compare 2 control register
	MOVE $t0, $zero
	SW $t0, ($s0)

	# Initialize the OC2R register before the output compare module, this register determins duty cycle
	LA $s0, OC2R
	LI $t0, 0x0180 # Shoot for 50% duty cycle, PR2 - 0x0200
	SW $t0, ($s0)
	# The OC1RS secondary output compare register will contain the actual duty cycle
	LA $s0, OC2RS
	LI $t0, 0x0180 # Shoot for 50% duty cycle
	SW $t0, ($s0)

	# Now configure the compare module using OC2CON
	# Bits 2:0 - 110 = PWM mode on OC2, 011 = compare event toggles OC2 pin
	# Bit 3 - 1 = Timer 3 clk src, 0 = Timer 2 clk src
	# Bit 5 - 1 = 32-bit comparisons, 0 = 16-bit comparisons
	# Bit 15 - 1 = enable output compare, 0 = disabled, not drawing current
	LA $s0, OC2CON
	MOVE $t0, $zero
	ORI $t0, $t0, 6 # PWM mode
	ORI $t0, $t0, 1 << 15 # Enable output compare module
	SW $t0, ($s0)

	# Set priority of compare match interrupt IPC2<20:18>
	LA $s0, IPC2SET
	LI $t0, 10 # priority 10
	SLL $t0, $t0, 18
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupOutputCompare2Module

.ENT check_sensors
check_sensors:
    LW $s0, (PORTB)
    ANDI $s0, $s0, 0xF
    LI $t0, 0b1001 # Straight
    LI $t1, 0b1101 # Left curve
    LI $t2, 0b1000 # Left turn
    LI $t3, 0b1011 # Right curve
    LI $t4, 0b0001 # Right turn
    LI $t5, 0b0000 # Circle

    BEQ $s0, $t0, straight
    BEQ $s0, $t1, left_curve
    BEQ $s0, $t2, left_turn
    BEQ $s0, $t3, right_curve
    BEQ $s0, $t4, right_turn
    BEQ $s0, $t5, stop

    LI $t0, 0b1100 # Might
    LI $t1, 0b1110 # Might
    LI $t2, 0b0011 # Might
    LI $t3, 0b0111 # Might
    LI $t4, 0b1111 # Might

    BEQ $s0, $t0, left_curve
    BEQ $s0, $t1, left_turn
    BEQ $s0, $t2, right_curve
    BEQ $s0, $t3, right_turn
    BEQ $s0, $t4, stop

    straight:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LI $t0, 1 << 7
        SW $t0, LATDCLR

        LI $t0, 1 << 6
        SW $t0, LATDSET

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        J end
    left_curve:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC2R
        LI $t0, 0x00AF # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC2RS
        LI $t0, 0x00AF # Shoot for 50% duty cycle
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC3R
        LI $t0, 0x0100 # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC3RS
        LI $t0, 0x0100 # Shoot for 50% duty cycle
        SW $t0, ($s0)

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        J end
    left_turn:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LI $t0, 1 << 7
        SW $t0, LATDSET

        LI $t0, 1 << 6
        SW $t0, LATDSET

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        J end
    right_curve:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC2R
        LI $t0, 0x0100 # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC2RS
        LI $t0, 0x0100 # Shoot for 50% duty cycle
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC3R
        LI $t0, 0x00AF # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC3RS
        LI $t0, 0x00AF # Shoot for 50% duty cycle
        SW $t0, ($s0)

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        J end
    right_turn:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LI $t0, 1 << 7
        SW $t0, LATDCLR

        LI $t0, 1 << 6
        SW $t0, LATDCLR

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)
        J end
    stop:
        LA $s0, OC2CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 0 << 15 # Disable output compare module
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC2R
        LI $t0, 0x0000 # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC2RS
        LI $t0, 0x0000 # Shoot for 50% duty cycle
        SW $t0, ($s0)

        # Initialize the OC1R register before the output compare module, this register determins duty cycle
        LA $s0, OC3R
        LI $t0, 0x0000 # Shoot for 50% duty cycle, PR2 - 0x0200
        SW $t0, ($s0)
        # The OC1RS secondary output compare register will contain the actual duty cycle
        LA $s0, OC3RS
        LI $t0, 0x0000 # Shoot for 50% duty cycle
        SW $t0, ($s0)

        LA $s0, OC2CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        LA $s0, OC3CON
        ORI $t0, $t0, 1 << 15 # Enable output compare module
        SW $t0, ($s0)

        J end

    end:
.END check_sensors

.ENT setupUART1
setupUART1:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# U1MODE - UART1 Mode Register, i.e. control/config register
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U1MODE
	LI $t0, 0 # disable UART1, reset it
	SW $t0, ($s0)

	# Clear the transmit and receive buffers
	LA $s0, U1TXREG # UART1 transmit register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)
	LA $s0, U1RXREG # UART1 receive register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)

	# Setup the baud rate - bits per second (bps) of data received and transmitted
	LA $s0, U1BRG # UART1 Baud Rate Register
	LI $t0, 259 # U1BRG = (PBCLK / (16 * baud rate)) - 1; PBCLK = 40 MHz, desired baud rate = 9600 bps
	SW $t0, ($s0)


	# U1STA - UART1 Status and Control Register
	# Bits 15:14 - UTXISEL TX Interrupt Mode Selection bits 10 = interrupt generated tranmit buffer becomes empty, 01 = interrupt is generated when all character transmitted,
	#                                                       00 = interrupt generated when transmit buffer becomes not full
	# Bit 12 - URXEN Receive Enable bit 1 = UART1 receiver is enabled, U1RX pin controlled by UART1; 0 = UART1 receiver disabled, U1RX pin ignored
	# Bit 10 - UXTEN Transmit Enable bit 1 = UART1 transmitter enabled, U1TX pin controlled by UART1; 0 = UART1 trasmitter diabled, transmissin aborted, buffer reset
	# Bit 9 - UTXBF Transmit Buffer Full Status bit 1 = Transmit buffer is full, 0 = not full
	# Bit 8 - TRMT Transmit Shift Registr is Empty bit 1 = Transmit shift register and transmit buffer empty, 0 = Transmit shift register is not empty, transmission in progress
	# Bits 7:6 - URXISEL Receive Interrupt Mode Selection bit 11 = Interrup flag bit set when receive buffer is full, 10 set when buffer 3/4 full, 0x flag bit set when character is received
	# Bit 3 - Parity Error Status bit 1 = parity error detected, 0 = parity error not detected
	# Bit 2 - FERR Framing Error Status bit 1 = framing error detected, 0 = no framing error detected
	# Bit 1 - OERR Receive Buffer Overrun Error Status bit 1 = receive buffer overflowed, 0 = buffer has not overflowed
	# Bit 0 - URXDA Receive Buffer Data Available bit 1 = receive buffer has data, 0 = receive buffer is empty
	LA $s0, U1STA
	LI $t0, 1 << 10 # Don't need to preserve any bits, overwrite with constant; not applying interrupts; enable transmission only
	SW $t0, ($s0)

	# Set priority
	# IPC6 <4:2> IEC0<27> = U1RX, IEC0<28> = U1TX

	# U1MODE - UART1 Mode Register, i.e. control/config register
	# Bit 0 - STSEL Stop Selection bit 1 = 2 stop bits, 0 = 1 stop bit
	# Bits 2:1 - PDSEL Parity and Data Selection bits 11 = 9 data, no parity; 10 = 8 data, odd parity; 01 = 8 data, even parity; 00 = 8 data, no parity
	# Bit 3 - BRGH High Baud Rate Enable bit 1 = high speed, 0 = standard speed
	# Bits 9:8 - UEN UART Enable bits 00 = U1TX and U1RX pins enabled and used
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U1MODE
	LI $t0, 0b1000000000000000 # Don't need to preserve any bits, overwrite with constant
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupUART1
