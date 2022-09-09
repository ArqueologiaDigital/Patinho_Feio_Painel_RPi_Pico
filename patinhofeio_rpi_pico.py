# (c) 2022 Felipe Correa da Silva Sanches <juca@members.fsf.org>
# Licensed under the GNU General Public License, version 2 (or later)
#
# This is a panel controller meant to be hooked to
# an emulator of the Patinho Feio computer.
#
# More info about this early 70's Brazilian pioneer computer
#  is available at https://github.com/felipesanches/PatinhoFeio/
#
# This sketch interfaces with a replica
# of the computer front-panel.
#
# The front panel contains:
# * 12 toggle-switches
# * a couple selectors with 2 positions each
# * 10 push-buttons (9 of which had gree LEDs as well)
# * 9 white LEDs
# * 62 red LEDs
#
# The LEDs are controlled by two MAX7219 modules that control 8x8 LEDs each.
#
# The push-buttons and toggle-switches are arranged in a 4x6 matrix of inputs.
#
# There's also a dot-matrix printer centronics interface to simulate
# the output of a teletype or DECWriter.

# printer pinout
# GND - marrom escuro
# STROBE - pin 1 - preto 
# D0 - branco
# D1 - amarelo
# D2 - rosa
# D3 - vermelho
# D4 - laranja
# D5 - marrom claro
# D6 - roxo claro
# D7 - azul claro
# BUSY - pin 11 - azul médio

import max7219
import math
from machine import Pin, SPI
from time import sleep

CENTRONICS_BUSY = 2
REGISTER_CLK = 3
DATA_PIN = 5
CS_PIN = 6
CLK_PIN = 7

SWITCH_ROWS = [Pin(n, Pin.IN) for n in [6, 7, 8, 9]]
SWITCH_COLUMNS = [Pin(n, Pin.OUT) for n in [10, 11, 12, 13, 14, 15]]

spi = SPI(0,sck=Pin(2),mosi=Pin(3))
cs = Pin(5, Pin.OUT)

display = max7219.Matrix8x8(spi, cs, 2)

display.brightness(1)   # 1 to 15
display.fill(1)
display.show()

inputs = []
old_inputs = []
for row in range(4):
    inputs.append([False] * 6)
    old_inputs.append([False] * 6)

leds = []
for row in range(8):
    leds.append([0] * 16)

def printer_writeByte(c):
    while digitalRead(CENTRONICS_BUSY) == HIGH: # wait for busy to go low
        pass

    mx.payloadWrite(0, c) # CHAR DATA

    mx.payloadWrite(1, 0x00) # STROBE LOW (active low)
    mx.spiSend()
    digitalWrite(REGISTER_CLK, HIGH)
    digitalWrite(REGISTER_CLK, LOW)

    mx.payloadWrite(1, 0xFF) # STROBE HIGH
    mx.spiSend()
    digitalWrite(REGISTER_CLK, HIGH)
    digitalWrite(REGISTER_CLK, LOW)


INIT_PRINTER_DEFAULTS = '@'
BOLD = 'E'
LEFT_MARGIN = 'l'
RIGHT_MARGIN = 'Q'

def escape_code(code):
    printer_writeByte(27)
    printer_writeByte(code)


def set_immediate_print_mode():
    escape_code('i')
    printer_writeByte(1)


def left_margin(n_chars):
    escape_code(LEFT_MARGIN)
    printer_writeByte(n_chars)


def right_margin(n_chars):
    escape_code(RIGHT_MARGIN)
    printer_writeByte(n_chars)


def printer_writeString(string):
    for c in string:
        printer_writeByte(c)


def initial_printer_commands():
    escape_code(INIT_PRINTER_DEFAULTS)

    # set unit of line spacing to the minimum vertical increment necessary
    # ...
  
    # set the printing area
    left_margin(5)
    right_margin(5)

    # select the font
    escape_code(BOLD)
  
    # set the printing position
    # ...

    # send one line's print data + CR + LF
    printer_writeString("IMPRESSORA OK\r\n")

    # end page with FF
    # ... escape_code(FORM_FEED)

    # end printing with '@'
    # escape_code(INIT_PRINTER_DEFAULTS)




def read_switch_matrix():
    for col in range(6):
        for j in range(6):
            SWITCH_COLUMNS[j](j == col)
        sleep(1)
        for i in range(4):
            inputs[i][col] = (SWITCH_ROWS[i]() == 1)



"""
class Interface {
private:
  bool controle;
  bool estado;
  bool pedido_de_interrupcao;
  bool permite_interrupcao;

public:
  Interface();
  virtual void write(uint8_t value);
  uint8_t read();

  uint8_t registrador;
};

#define READY true
#define BUSY false

Interface::Interface(){
  this->registrador = 0x00;
  this->controle = false;
  this->estado = READY;
  this->pedido_de_interrupcao = false;
  this->permite_interrupcao = false;
}

void Interface::write(uint8_t value){
  this->registrador = value;
}

uint8_t Interface::read(){
  return this->registrador;
}

Interface* canais[16];

class TeleType(Interface):
    def write(self, uint8_t value){
        this->registrador = value;
        # sleep(1000/10); // 10 chars per second. This is REALLY BAD emulation-wise but it looks nice :-)

        if (value=='\n')
            printer_writeByte('\r');

        printer_writeByte(value);
"""

INDEX_REG = 0
EXTENSION_REG = 1
RAM_SIZE = 4096

RAM = [0] * RAM_SIZE

_VAI_UM = False
_TRANSBORDO = False

_RE = 0               # 12-bit "Registrador de Endereço" = Address Register
_RD = 0               #  8-bit "Registrador de Dados" = Data Register
_RI = 0               #  8-bit "Registrador de Instrução" = Instruction Register
_ACC = 0              #  8-bit "Acumulador" = Accumulator Register
_CI = 0               # 12-bit "Contador de Instrução" = Instruction Counter
_DADOS_DO_PAINEL = 0  # 12-bit of data provided by the user
                      # via panel toggle-switches

_FASE = 0  # Determines which cycle of a cpu instruction
           # we are currently executing

_PARADO = True  # CPU is stopped.
_EXTERNO = False # CPU is waiting interrupt.
_MODO = 0  # CPU operation modes:
indirect_addressing = False
scheduled_IND_bit_reset = False
memoria_protegida = False
enderecamento_sequencial = False

NORMAL = 1           # normal execution
CICLO_UNICO = 2      # single-cycle
INSTRUCAO_UNICA = 3  # single-instruction
ENDERECAMENTO = 4    # addressing mode
ARMAZENAMENTO = 5    # data write mode
EXPOSICAO = 6        # data read mode

def read_index_reg():
    return RAM[INDEX_REG]


def write_index_reg(value):
    RAM[INDEX_REG] = value


def DADOS_DO_PAINEL(value):
    _DADOS_DO_PAINEL = value
    map_col = [13, 8, 10, 9, 13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [ 2, 2,  2, 2,  0, 0,  0, 0,  0,  0,  0,  0]
    for i in range(12):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def VAI_UM(value):
    _VAI_UM = value
    leds[2][15] = value


def TRANSBORDO(value):
    _TRANSBORDO = value
    leds[2][14] = value


def PARADO(value):
    # This represents that the CPU is stopped.
    # Only a startup command ("PARTIDA") at execution modes
    # (NORMAL, SINGLE-INST, or SINGLE-CYCLE) can restart it.
    _PARADO = value
    leds[2][12] = value


def EXTERNO(value):
    # This represents that the CPU is stopped
    # waiting for an interrupt from an external device.
    _EXTERNO = value
    leds[2][11] = value


def CI(value):
    _CI = value
    map_col = [14, 15, 12, 11, 13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [ 7,  7,  7,  7,  1, 1,  1, 1,  1,  1,  1,  1]
    for i in range(12):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def RE(value):
    _RE = value
    map_col = [13, 8, 10, 9, 13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [ 7, 7,  7, 7,  3, 3,  3, 3,  3,  3,  3,  3]
    for i in range(12):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def sssRD(value):
    _RD = value
    map_col = [3,  4,  7,  6,  1,  2,  0,  5]
    map_row = [4,  4,  4,  4,  4,  4,  4,  4]
    for i in range(8):
        leds[map_row[7-i]][(8+map_col[7-i])%16] = bool(value & (1 << i))

def RD(value):
    _RD = value
    map_col = [13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [ 4, 4,  4, 4,  4,  4,  4,  4]
    for i in range(8):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def RI(value):
    _RI = value
    map_col = [13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [6,  6,  6,  6,  6,  6,  6,  6]
    for i in range(8):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def ACC(value):
    _ACC = value
    map_col = [13, 8, 10, 9, 14, 15, 12, 11]
    map_row = [ 5, 5,  5, 5,  5,  5,  5,  5]
    for i in range(8):
        leds[map_row[i]][map_col[i]] = bool(value & (1 << i))


def FASE(value):
    _FASE = value
    map_col = [3, 5, 6, 0, 4, 7, 1]
    map_row = [0, 0, 0, 0, 0, 0, 0]
    for i in range(7):
        leds[map_row[i]][map_col[i]] = (_FASE == i+1)


def MODO(value):
    _MODO = value
    map_col = [3, 5, 6, 0, 4, 7]
    map_row = [2, 2, 2, 2, 2, 2]
    for i in range(6):
        leds[map_row[i]][map_col[i]] = (_MODO == i+1)


def LED_ESPERA(value):
    # I think this button did not really have a lamp
    pass


def LED_INTERRUPCAO(value):
    # I think this button did not really have a lamp
    pass


def LED_PREPARACAO(value):
    # I think this button did not really have a lamp
    pass


"""
void reset_CPU(){
  VAI_UM(false);
  TRANSBORDO(false);
  CI(0x000);
  RE(0x000);
  RD(0x00);
  RI(0x00);
  ACC(0x00);
  DADOS_DO_PAINEL(0x000);
  FASE(1);
  PARADO(true);
  EXTERNO(false);
  MODO(NORMAL);
  LED_ESPERA(false);
  LED_INTERRUPCAO(false);
  LED_PREPARACAO(false);
  indirect_addressing = false;
  scheduled_IND_bit_reset = false;
}

void load_example_hardcoded_program(){
  /*
    HELLO WORLD PROGRAM that prints
    "PATINHO FEIO" to the teletype:
  */

  RAM[0x06] = 0x80; //006: 80     LIMPO
  RAM[0x07] = 0x9e; //007: 9E     TRI
  RAM[0x08] = 0x50; //008: 50 1C  CARX (IDX) + /01C
  RAM[0x09] = 0x1c;
  RAM[0x0A] = 0xca; //00A: CA 80  SAI /A0             // output ACC value to the teletype (channel A)
  RAM[0x0B] = 0x80;
  RAM[0x0C] = 0xca; //00C: CA 21  SAL /A1             // channel A (teletype), function 1 (check READY flag)
  RAM[0x0D] = 0x21;
  RAM[0x0E] = 0x00; //00E: 00 0C  PLA /00C            // Jump back to previous instruction (unless the teletype READY
  RAM[0x0F] = 0x0c;                                   //   flag causes this instruction to be skipped).
  RAM[0x10] = 0x9e; //010: 9E     TRI                 // TRI = Exchange values of IND reg and ACC reg.
  RAM[0x11] = 0x85; //011: 85     INC                 // Increment ACC
  RAM[0x12] = 0x20; //012: 20 00  ARM (IDX)           // Store ACC value into the IDX register.
  RAM[0x13] = 0x00;
  RAM[0x14] = 0x60; //014: 60 1B  SOM /01B            // Subtract the length of the string from the ACC value (see DB -14 below. "SOM" means "add")
  RAM[0x15] = 0x1B;
  RAM[0x16] = 0xA0; //016: A0 08  PLAN /008           // Conditional jump to /008 (jumps if ACC is negative)
  RAM[0x17] = 0x08;
  RAM[0x18] = 0x9D; //018: 9D     PARE                // Halt the CPU. Can be restarted by manually pushing the PARTIDA (startup) panel button.
  RAM[0x19] = 0x00; //019: 00 06  PLA /006            // If you restart the CPU, this is the next instruction, which jumps straight back to the
  RAM[0x1A] = 0x06;                                   //   routine entry point, effectively causing the whole program to run once again.
  RAM[0x1B] = 0xF2; //01B: F2     DB -14              // This is the 2's complement for -len(string)
  RAM[0x1C] = 'P';  //01C: "PATINHO FEIO\n"           // This is the string.
  RAM[0x1D] = 'A';
  RAM[0x1E] = 'T';
  RAM[0x1F] = 'I';
  RAM[0x20] = 'N';
  RAM[0x21] = 'H';
  RAM[0x22] = 'O';
  RAM[0x23] = ' ';
  RAM[0x24] = 'F';
  RAM[0x25] = 'E';
  RAM[0x26] = 'I';
  RAM[0x27] = 'O';
  RAM[0x28] = 0x0D;
  RAM[0x29] = 0x0A;

  //Entry point:
  CI(0x006);
}
"""

def emulator_init():
    # initial_printer_commands()

    for i in range(RAM_SIZE):
        RAM[i] = 0

    reset_CPU()
    load_example_hardcoded_program()


def old_setup():
    # Serial.begin(9600)

    # for c in range(16):
    #    canais[c] = None

    # canais[0x6] = new Duplex8bits()
    # canais[0x7] = new Duplex8bits()
    ##### canais[0xA] = new TeleType()
    # canais[0xB] = new DECWriter()
    # canais[0xE] = new LeitoraDeFita()

    mx.begin()
    mx.clear()
    mx.payloadWrite(0, 0x00) # CHAR DATA  
    mx.payloadWrite(1, 0xFF) # STROBE HIGH (active low)
    mx.spiSend()

    pinMode(CENTRONICS_BUSY, INPUT)
    pinMode(REGISTER_CLK, OUTPUT)
    digitalWrite(REGISTER_CLK, LOW)
    sleep(100)
    digitalWrite(REGISTER_CLK, HIGH)
    sleep(100)
    digitalWrite(REGISTER_CLK, LOW)



count = 0
t = 0.0
def register_LEDs_demo():
    global count
    global t
    # Let's do a sine-wave effect on
    # the 12-bit panel data register:
    value = 0
    for i in range(12):
        if ((i + 0.5) < (6 + 6 * math.sin(t * 360))):
            value |= (1 << i)

    t += 0.001
    DADOS_DO_PAINEL(value)

    # The address register will have a mirrored sine-wave.
    # That's why we flip the bits here:
    RE(~value)

    ACC(int(value/16))
    RI(int(value/16))
    RD(int(value/16))

    # And let's display an incremental count at
    # the instruction counter register:
    CI(count)

    # While the overflow and carry bits will
    # alternate their blinking:
    VAI_UM((count / 10) % 2 == 0)
    TRANSBORDO((count / 10) % 2 == 1)

    # And the same kind of effect for these lights:
    PARADO((count / 6) % 2 == 0)
    EXTERNO((count / 6) % 2 == 1)

    # These will blink a lot :-P
    LED_ESPERA(count % 2 == 0)
    LED_PREPARACAO(count % 2 == 0)
    LED_INTERRUPCAO(count % 2 == 0)

    # Finally, these sets of lights will turn on sequentially:
    FASE(1 + int(count / 5) % 7)
    MODO(1 + int(count / 10) % 6)

    count = (count+1) % (12*10*15*1000) # This wrap-around value is a multiple of
                                        # the period of all demo-effects


"""
byte read_ram(int addr){
  return RAM[addr];
}

void write_ram(int addr, byte value){
  RAM[addr % RAM_SIZE] = value;
}

void inc_CI(){
  CI((_CI+1)%RAM_SIZE);
}

unsigned int compute_effective_address(unsigned int addr){
  if (indirect_addressing){
    addr = (read_ram(addr+1) << 8) | read_ram(addr);
    if (addr & 0x1000)
      return compute_effective_address(addr & 0xFFF);
  }
  return addr;
}

byte opcode;

/*******************************
CPU Instructions implementation
********************************/

void PLA_instruction(){
  /* OPCODE: 0x0X */
  //PLA = "Pula": Jump to address
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  addr = compute_effective_address(addr);
  inc_CI();
  CI(addr);
}

void PLAX_instruction(){
  /* OPCODE: 0x1X */
  //PLAX = "Pula indexado": Jump to indexed address
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  inc_CI();
  byte idx = read_index_reg();
  addr = compute_effective_address(idx + addr);
  CI(addr);
}

void ARM_instruction(){
  /* OPCODE: 0x2X */
  //ARM = "Armazena": Store the value of the accumulator into a given memory position
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  addr = compute_effective_address(addr);
  inc_CI();
  write_ram(addr, _ACC);
}

void ARMX_instruction(){
  /* OPCODE: 0x3X */
  //ARMX = "Armazena indexado": Store the value of the accumulator into a given indexed memory position
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  inc_CI();
  byte idx = read_index_reg();
  addr = compute_effective_address(idx + addr);
  write_ram(addr, _ACC);
}

void CAR_instruction(){
  /* OPCODE: 0x4X */
  //CAR = "Carrega": Load a value from a given memory position into the accumulator
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  inc_CI();
  addr = compute_effective_address(addr);
  ACC(read_ram(addr));
}

void CARX_instruction(){
  /* OPCODE: 0x5X */
  //CARX = "Carga indexada": Load a value from a given indexed memory position into the accumulator
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  inc_CI();
  byte idx = read_index_reg();
  addr = compute_effective_address(idx + addr);
  ACC(read_ram(addr));
}

void SOM_instruction(){
  /* OPCODE: 0x6X */
  //SOM = "Soma": Add a value from a given memory position into the accumulator
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  addr = compute_effective_address(addr);
  inc_CI();
  ACC(_ACC + read_ram(addr));
  //TODO: update V and T flags
}

void SOMX_instruction(){
  /* OPCODE: 0x7X */
  //SOMX = "Soma indexada": Add a value from a given indexed memory position into the accumulator
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  inc_CI();
  byte idx = read_index_reg();
  addr = compute_effective_address(idx + addr);
  ACC(_ACC + read_ram(addr));
  //TODO: update V and T flags
}

void LIMPO_instruction(){
  /* OPCODE: 0x80 */
  //LIMPO:
  //     Clear accumulator and flags
  ACC(0);
  TRANSBORDO(0);
  VAI_UM(0);
}

void UM_instruction(){
  /* OPCODE: 0x81 */
  //UM="One":
  //    Load 1 into accumulator
  //    and clear the flags
  ACC(1);
  TRANSBORDO(0);
  VAI_UM(0);
}

void CMP1_instruction(){
  /* OPCODE: 0x82 */
  //CMP1:
  // Compute One's complement of the accumulator
  //    and clear the flags
  ACC(~_ACC);
  TRANSBORDO(0);
  VAI_UM(0);
}

void CMP2_instruction(){
  /* OPCODE: 0x83 */
  //CMP2:
  // Compute Two's complement of the accumulator
  //    and updates flags according to the result of the operation
  ACC(~_ACC + 1);

  //TODO: fix-me (I'm not sure yet how to compute the flags here):
  TRANSBORDO(0);
  VAI_UM(0);
}

void LIM_instruction(){
  /* OPCODE: 0x84 */
  //LIM="Limpa":
  // Clear flags
  TRANSBORDO(0);
  VAI_UM(0);
} 

void INC_instruction(){
  /* OPCODE: 0x85 */
  //INC:
  // Increment accumulator
  ACC(_ACC+1);
  TRANSBORDO(0); //TODO: fix-me (I'm not sure yet how to compute the flags here)
  VAI_UM(0); //TODO: fix-me (I'm not sure yet how to compute the flags here)
}

void UNEG_instruction(){
  /* OPCODE: 0x86 */
  //UNEG="Um Negativo":
  // Load -1 into accumulator and clear flags
  ACC(-1);
  TRANSBORDO(0);
  VAI_UM(0);
}

void LIMP1_instruction(){
  /* OPCODE: 0x87 */
  //LIMP1:
  //    Clear accumulator, reset T and set V
  ACC(0);
  TRANSBORDO(0);
  VAI_UM(1);
}

void PNL_0_instruction(){
  /* OPCODE: 0x88 */
  //PNL 0:
  ACC(_DADOS_DO_PAINEL & 0xFF);
  TRANSBORDO(0);
  VAI_UM(0);
}

void PNL_1_instruction(){
  /* OPCODE: 0x89 */
  //PNL 1:
  ACC((_DADOS_DO_PAINEL & 0xFF) + 1);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_2_instruction(){
  /* OPCODE: 0x8A */
  //PNL 2:
  ACC((_DADOS_DO_PAINEL & 0xFF) - _ACC - 1);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_3_instruction(){
  /* OPCODE: 0x8B */
  //PNL 3:
  ACC((_DADOS_DO_PAINEL & 0xFF) - _ACC);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_4_instruction(){
  /* OPCODE: 0x8C */
  //PNL 4:
  ACC((_DADOS_DO_PAINEL & 0xFF) + _ACC);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_5_instruction(){
  /* OPCODE: 0x8D */
  //PNL 5:
  ACC((_DADOS_DO_PAINEL & 0xFF) + _ACC + 1);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_6_instruction(){
  /* OPCODE: 0x8E */
  //PNL 6:
  ACC((_DADOS_DO_PAINEL & 0xFF) - 1);
  //TODO: TRANSBORDO(?);
  //TODO: VAI_UM(?);
}

void PNL_7_instruction(){
  /* OPCODE: 0x8F */
  //PNL 7:
  ACC(_DADOS_DO_PAINEL & 0xFF);
  VAI_UM(1);
}

void ST_0_instruction(){
  /* OPCODE: 0x90 */
  //ST 0 = "Se T=0, Pula"
  //       If T is zero, skip the next instruction
  if (_TRANSBORDO == 0){
    inc_CI(); //skip
  }
}

void STM_0_instruction(){
  /* OPCODE: 0x91 */
  //STM 0 = "Se T=0, Pula e muda"
  //        If T is zero, skip the next instruction
  //        and toggle T.
  if (_TRANSBORDO == 0){
    inc_CI(); //skip
    TRANSBORDO(1);
  }
}

void ST_1_instruction(){
  /* OPCODE: 0x92 */
  //ST 1 = "Se T=1, Pula"
  //       If T is one, skip the next instruction
  if (_TRANSBORDO == 1){
    inc_CI(); //skip
  }
}

void STM_1_instruction(){
  /* OPCODE: 0x93 */
  //STM 1 = "Se T=1, Pula e muda"
  //        If T is one, skip the next instruction
  //        and toggle T.
  if (_TRANSBORDO == 1){
    inc_CI(); //skip
    TRANSBORDO(0);
  }
}

void SV_0_instruction(){
  /* OPCODE: 0x94 */
  //SV 0 = "Se V=0, Pula"
  //       If V is zero, skip the next instruction
  if (_VAI_UM == 0){
    inc_CI(); //skip
  }
}

void SVM_0_instruction(){
  /* OPCODE: 0x95 */
  //SVM 0 = "Se V=0, Pula e muda"
  //        If V is zero, skip the next instruction
  //        and toggle V.
  if (_VAI_UM == 0){
    inc_CI(); //skip 
    VAI_UM(1);
  } 
} 

void SV_1_instruction(){
  /* OPCODE: 0x96 */
  //SV 1 = "Se V=1, Pula"
  //       If V is one, skip the next instruction
  if (_VAI_UM == 1){
    inc_CI(); //skip 
  } 
} 

void SVM_1_instruction(){
  /* OPCODE: 0x97 */
  //SVM 1 = "Se V=1, Pula e muda"
  //        If V is one, skip the next instruction
  //        and toggle V.
  if (_VAI_UM == 1){
    inc_CI(); //skip 
    VAI_UM(0);
  } 
}

void PUL_instruction(){
  /* OPCODE: 0x98 */
  //PUL="Pula para /002 a limpa estado de interrupcao"
  //     Jump to address /002 and disables interrupts
  CI(0x002);
  EXTERNO(0); //TODO: verify this!
}

void TRE_instruction(){
  /* OPCODE: 0x99 */
  //TRE="Troca conteudos de ACC e EXT"
  //     Exchange the value of the accumulator with the ACC extension register
  byte value = _ACC;
  ACC(read_ram(EXTENSION_REG));
  write_ram(EXTENSION_REG, value);
}

void INIB_instruction(){
  /* OPCODE: 0x9A */
  //INIB="Inibe"
  //     disables interrupts
  //TODO: Implement-me!  m_interrupts_enabled = false;
}

void PERM_instruction(){
  /* OPCODE: 0x9B */
  //PERM="Permite"
  //     enables interrupts
  //TODO: Implement-me!  m_interrupts_enabled = true;
}

void ESP_instruction(){
 /* OPCODE: 0x9C */
 //ESP="Espera":
 //    Holds execution and waits for an interrupt to occur.
 //TODO:  m_run = false;
 //TODO:  m_wait_for_interrupt = true;
}

void PARE_instruction(){
  /* OPCODE: 0x9D */
  //PARE="Pare":
  //    Holds execution. This can only be recovered by
  //    manually triggering execution again by
  //    pressing the "Partida" (start) button in the panel
  PARADO(true);
  EXTERNO(false);
}

void TRI_instruction(){
  /* OPCODE: 0x9E */
  //TRI="Troca com Indexador":
  //     Exchange the value of the accumulator with the index register
  byte value = _ACC;
  ACC(read_index_reg());
  write_index_reg(value);
}

void PLAN_instruction(){
  /* OPCODE: 0xAX */
  //PLAN = "Pula se ACC negativo": Jump to a given address if ACC is negative
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  addr = compute_effective_address(addr);
  inc_CI();
  if ((signed char) _ACC < 0){
    CI(addr);
  }
}

void PLAZ_instruction(){
  /* OPCODE: 0xBX */
  //PLAZ = "Pula se ACC for zero": Jump to a given address if ACC is zero
  unsigned int addr = (opcode & 0x0F) << 8 | read_ram(_CI);
  addr = compute_effective_address(addr);
  inc_CI();
  if (_ACC == 0){
    CI(addr);
  }
}

void FNC_instruction(byte channel, byte function){
  /* OPCODE: 0xCX 0x1X */
  //Implement-me!
}

void SAL_instruction(byte channel, byte function){
  /* OPCODE: 0xCX 0x2X */
  //SAL="Salta"
  //    Skips a couple bytes if a condition is met
  bool skip = false;
  switch(function){
    case 1:
      //TODO: implement-me! skip = (m_iodev_status[channel] == IODEV_READY);
      skip = true;
      break;
    case 2: 
      /* TODO:
         skip = false;
         if (! m_iodev_is_ok_cb[channel].isnull()
            && m_iodev_is_ok_cb[channel](0))
      */
      skip = true;
      break;
    case 4:
      /*TODO:
         skip =false;
         if (! m_iodev_IRQ_cb[channel].isnull()
            && m_iodev_IRQ_cb[channel](0) == true)
      */
      skip = true;
      break;
  }
  if (skip){
    inc_CI();
    inc_CI();
  }
}

void ENTR_instruction(byte channel){
  /* OPCODE: 0xCX 0x4X */
  /* ENTR = "Input data from I/O device" */
  //TODO: handle multiple device channels: m_iodev_write_cb[channel](ACC);
  //Implement-me!
}

void SAI_instruction(byte channel){
  /* OPCODE: 0xCX 0x8X */
  /* SAI = "Output data to I/O device" */
  if (canais[channel])
    canais[channel]->write(_ACC);
}

void IO_instructions(){
  //Executes I/O functions
  //TODO: Implement-me!
  byte value = read_ram(_CI);
  inc_CI();
  byte channel = opcode & 0x0F;
  byte function = value & 0x0F;
  switch(value & 0xF0){
    case 0x10: FNC_instruction(channel, function); return;
    case 0x20: SAL_instruction(channel, function); return;
    case 0x40: ENTR_instruction(channel); return;
    case 0x80: SAI_instruction(channel); return;
  }
}

void SUS_instruction(){
  /* OPCODE: 0xEX */
  //TODO: Implement-me!
}

void PUG_instruction(){
  /* OPCODE: 0xFX */
  //TODO: Implement-me!
}

void XOR_instruction(){
  /* OPCODE: 0xD2 */
  //XOR: Computes the bitwise XOR of an immediate into the accumulator
  ACC(_ACC ^ read_ram(_CI));
  inc_CI();
  //TODO: update T and V flags
}

void NAND_instruction(){
  /* OPCODE: 0xD4 */
  //NAND: Computes the bitwise XOR of an immediate into the accumulator
  ACC(~(_ACC & read_ram(_CI)));
  inc_CI();
  //TODO: update T and V flags
}

void SOMI_instruction(){
  /* OPCODE: 0xD8 */
  //SOMI="Soma Imediato":
  //Add an immediate into the accumulator
  //TODO: set_flag(V, ((((int16_t) ACC) + ((int16_t) READ_BYTE_PATINHO(PC))) >> 8));
  //TODO: set_flag(T, ((((int8_t) (ACC & 0x7F)) + ((int8_t) (READ_BYTE_PATINHO(PC) & 0x7F))) >> 7) == V);
  ACC(_ACC + read_ram(_CI));
  inc_CI();
}

void CARI_instruction(){
  /* OPCODE: 0xDA */
  //CARI="Carrega Imediato":
  //     Load an immediate into the accumulator
  ACC(read_ram(_CI));
  inc_CI();
}

void IND_instruction(){
  indirect_addressing = true;
}

void shift_rotate_instructions(){
  //TODO: Implement-me!
}

void run_one_instruction(){
  unsigned int tmp;
  byte value, channel, function;
  opcode = read_ram(_CI);
  RI(opcode); //para mostrar o opcode no painel

  if (scheduled_IND_bit_reset)
    indirect_addressing = false;

  if (indirect_addressing)
    scheduled_IND_bit_reset = true;


  #ifdef DEBUG
  Serial.print("CI: ");
  Serial.print(_CI, HEX);
  Serial.print(" OPCODE: ");
  Serial.print(opcode, HEX);
  Serial.print(" Mascarado: ");
  Serial.println(opcode & 0xF0, HEX);
  #endif

  inc_CI();

  switch (opcode){
    case 0x80: LIMPO_instruction(); return;
    case 0x81: UM_instruction(); return;
    case 0x82: CMP1_instruction(); return;
    case 0x83: CMP2_instruction(); return;
    case 0x84: LIM_instruction(); return;
    case 0x85: INC_instruction(); return;
    case 0x86: UNEG_instruction(); return;
    case 0x87: LIMP1_instruction(); return;
    case 0x88: PNL_0_instruction(); return;
    case 0x89: PNL_1_instruction(); return;
    case 0x8A: PNL_2_instruction(); return;
    case 0x8B: PNL_3_instruction(); return;
    case 0x8C: PNL_4_instruction(); return;
    case 0x8D: PNL_5_instruction(); return;
    case 0x8E: PNL_6_instruction(); return;
    case 0x8F: PNL_7_instruction(); return;
    case 0x90: ST_0_instruction(); return;
    case 0x91: STM_0_instruction(); return;
    case 0x92: ST_1_instruction(); return;
    case 0x93: STM_1_instruction(); return;
    case 0x94: SV_0_instruction(); return;
    case 0x95: SVM_0_instruction(); return;
    case 0x96: SV_1_instruction(); return;
    case 0x97: SVM_1_instruction(); return;
    case 0x98: PUL_instruction(); return;
    case 0x99: TRE_instruction(); return;
    case 0x9A: INIB_instruction(); return;
    case 0x9B: PERM_instruction(); return;
    case 0x9C: ESP_instruction(); return;
    case 0x9D: PARE_instruction(); return;
    case 0x9E: TRI_instruction(); return;
    case 0x9F: IND_instruction(); return;
    case 0xD1: shift_rotate_instructions(); return;
    case 0xD2: XOR_instruction(); return;
    case 0xD4: NAND_instruction(); return;
    case 0xD8: SOMI_instruction(); return;
    case 0xDA: CARI_instruction(); return;
  }

  switch (opcode & 0xF0){
    case 0x00: PLA_instruction(); return;
    case 0x10: PLAX_instruction(); return;
    case 0x20: ARM_instruction(); return;
    case 0x30: ARMX_instruction(); return;
    case 0x40: CAR_instruction(); return;
    case 0x50: CARX_instruction(); return;
    case 0x60: SOM_instruction(); return;
    case 0x70: SOMX_instruction(); return;
    //case 0x80: single byte instructions
    //case 0x90:     declared above
    case 0xA0: PLAN_instruction(); return;
    case 0xB0: PLAZ_instruction(); return;
    case 0xC0: IO_instructions(); return;
    //case 0xD0: shift/rotate, XOR, NAND,
    //           SOMI, CARI instructions above
    case 0xE0: SUS_instruction(); return;
    case 0xF0: PUG_instruction(); return;
    default:
#ifdef DEBUG
      Serial.print("OPCODE INVALIDO: ");
      Serial.println(opcode, HEX);
#endif
      PARADO(true);
      return;
    }
}
"""

def emulator_loop():
    read_inputs()
  
    if not _PARADO:
        run_one_instruction()

    if _MODO in [CICLO_UNICO,
                 INSTRUCAO_UNICA]:
        PARADO(True)

    #while Serial.available():
    #    printer_writeByte(Serial.read())



def wait_for_buttons_release():
    pressed = True;
    while pressed:
        read_switch_matrix()
        pressed = False

        for i in range(6):
            if inputs[2][i]:
                pressed = True

        for i in range(4):
            if inputs[1][i]:
                pressed = True


def read_inputs():
    read_switch_matrix()

    # read the toggle switches and update DADOS_DO_PAINEL
    dados = 0
    for col in range(6):
        if inputs[0][col]: dados |= (1 << col)
        if inputs[3][col]: dados |= (1 << (col+6))

    DADOS_DO_PAINEL(dados)

    # when a mode button is pressed, set the corresponding mode:
    for col in range(6):
        if (inputs[2][col]):
            MODO(6-col)

    # chaves de modos de memória:
    enderecamento_sequencial = inputs[1][4]
    memoria_protegida = inputs[1][5]

    # botão "PREPARAÇÂO":
    if inputs[1][0]:
        reset_CPU()

    # botão "ESPERA":
    if inputs[1][3]:
        espera()

    # botão "INTERRUPÇÂO"
    if inputs[1][2]:
        interrupcao()

    # botão "PARTIDA":
    if inputs[1][1]:
         partida()


def espera():
    #TODO: Implement-me!  
    pass


def interrupcao():
    #TODO: Implement-me!  
    pass


def partida():
    if _MODO in [NORMAL,
                 CICLO_UNICO,
                 INSTRUCAO_UNICA]:
        PARADO(False)

    elif _MODO == ENDERECAMENTO:
        RE(_DADOS_DO_PAINEL)

    elif _MODO == ARMAZENAMENTO:
        if not (memoria_protegida and _RE >= 0xF80):
            dado = _DADOS_DO_PAINEL & 0xFF
            write_ram(_RE, dado)
            RD(dado)

        if enderecamento_sequencial:
            RE(_RE+1)

    elif _MODO == EXPOSICAO:
        RD(read_ram(_RE))
        if enderecamento_sequencial:
            RE(_RE+1)

    wait_for_buttons_release()


def send_LED_data():
    for row in range(8):
      for col in range(16):
          if leds[row][col]:
              display.pixel(col, row, 1)
          else:
              display.pixel(col, row, 0)
    display.show();

display.brightness(15)
display.fill(0)


while True:
    # emulator_loop();

    # This is the most complete blinking demo
    # which blinks every LED in the panel:
    register_LEDs_demo();
    sleep(0.1);

    send_LED_data()


