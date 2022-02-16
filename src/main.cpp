#include <Arduino.h>
//#include <Print.h>
//#include <HardwareSerial.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

// Valores para calibrar sensibilidade
#define triggerValue 6     // Se a variação entre a media e a ultima leitura não passar desse valor, o sensor não é considerado ativo
#define divisionValue 5.00 // Define de quantos valores a variavel media vai calcular o valor

// Funções basicas
#define NOP __asm__ __volatile__("nop\n\t") // Assembly nop
#define set_bit(y, bit) (y |= (1 << bit))
#define clr_bit(y, bit) (y &= ~(1 << bit))
#define cpl_bit(y, bit) (y ^= (1 << bit))
#define tst_bit(y, bit) (y & (1 << bit))

// Pinos de I/O
#define sendPin 7                     // Pino que envia o sinal do sensor capacitivo
uint8_t receivePin[4] = {2, 3, 4, 5}; // Pinos que recebem o sinal
#define led_WS2812 PB5                // Pino da fita de led

// Variaveis
unsigned long total;
double media[64];
char Rmove[4], Smove[4], Data = ' ';
uint8_t sCasa, fCasa, Rselected, Leds[222], MUX;
int value[64], eval;
bool pColor;

// Funções para comunicação Serial
void SerialBegin(unsigned long BAUD)
{
  unsigned int ubrr0 = F_CPU / 16 / BAUD - 1; // Calculo do valor ubrr0 de 11bits
  UBRR0H = (unsigned char)(ubrr0 >> 8);       // Ajusta a taxa de transmissão
  UBRR0L = (unsigned char)ubrr0;              //
  UCSR0A = 0;                                 // Desabilitar velocidade dupla (no Arduino é habilitado por padrão)
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);       // Habilita a transmissão e a recepção
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // Modo assíncrono, 8 bits de dados, 1 bit de parada, sem paridade
}
void SerialWrite(unsigned char dado)
{
  while (!(UCSR0A & (1 << UDRE0))) // Espera o transmissor estar pronto p/ enviar
    ;                              //
  UDR0 = dado;                     // Envia o dado
}
unsigned char SerialRead()
{
  while (!(UCSR0A & (1 << RXC0))) // Espera o dado ser recebido
    ;                             //
  return UDR0;                    // Eetorna o dado recebido
}

// Funções para controle dos Leds
void setLed()
{                             // Manda um bit 1 para o led
  set_bit(PORTB, led_WS2812); // LedPin HIGH
  NOP;                        // Espera 312.5ns
  NOP;
  NOP;
  NOP;
  NOP;
  clr_bit(PORTB, led_WS2812); // LedPin LOW
  NOP;                        // Espera 125ns
  NOP;
}
void clearLed()
{                             // Manda um bit 0 para o led
  set_bit(PORTB, led_WS2812); // LedPin HIGH
  NOP;                        // Espera 125ns
  NOP;
  clr_bit(PORTB, led_WS2812); // LedPin LOW
  NOP;                        // Espera 312.5ns
  NOP;
  NOP;
  NOP;
  NOP;
}
void LedShow()
{                               // Envia para o led todos os bits de todos os leds
  for (int i = 0; i < 222; i++) // Cada um dos 74 leds tem 3 bytes
    for (int j = 0; j < 8; j++) // Cada byte tem 8 bits
      if (tst_bit(Leds[i], j))  // Se for 1
        setLed();               // Manda bit 1
      else                      // Senão
        clearLed();             // Manda bit 0
  _delay_ms(1);                 // Espera 1ms por segurança
}
void corCasa(uint8_t casa, int red, int green, int blue)
{
  if (casa < 64) // Conversão para as casas do tabuleiro, ja que a ordem dos leds e das casas não é a mesma
  {
    if (casa / 8 % 2) // Se se for uma linha impar
    {
      int AUX = (63 - casa) * 3; // Conversão
      Leds[AUX] = green;         // Salvando a cor do led na sua variavel
      Leds[AUX + 1] = red;
      Leds[AUX + 2] = blue;
    }
    else // Se for uma linha par
    {
      int AUX = (64 - ((casa / 8 + 1) * 8) + casa % 8) * 3; // Conversão
      Leds[AUX] = green;                                    // Salvando a cor do led na sua variavel
      Leds[AUX + 1] = red;
      Leds[AUX + 2] = blue;
    }
  }
  else // São os leds da avaliação da posição
  {
    Leds[casa * 3] = green; // Salvando a cor do led na sua variavel
    Leds[casa * 3 + 1] = red;
    Leds[casa * 3 + 2] = blue;
  }
}

void trigger()
{
  uint8_t biggest = 0;                                                           // Zera a variavel que contem o maior valor dos sensores
  Rselected = 100;                                                               // Reseta a variavel que guarda o pino de maior valor, sendo 100 para nenhum
  for (int i = 0; i < 64; i++)                                                   // Varredura por todos os sensores
    if ((value[i] - media[i]) > triggerValue && biggest < (value[i] - media[i])) // Valor minimo para considerar algum movimento e teste de maior valor                                                               //
    {                                                                            //
      biggest = (value[i] - media[i]);                                           // Atualizando o maior valor
      Rselected = i;                                                             // Salvando o resultado
    }
}
long ReadCapacitiveSensor(uint8_t samples, uint8_t MUX)
{
  total = 0;                                // Zera a variavel que conterá o valor
                                            //
  for (uint8_t i = 0; i < samples; i++)     // Faz "sample" leituras, somando tudo em total
  {                                         // Faz a leitura do sensor
    set_bit(DDRD, receivePin[MUX]);         // Receive OUTPUT
    clr_bit(PORTD, receivePin[MUX]);        // Receive LOW
    _delay_us(3);                           // Espera 10ms
    clr_bit(DDRD, receivePin[MUX]);         // Receive INPUT
    clr_bit(PORTD, receivePin[MUX]);        // Receive PULLUP desligados
    set_bit(PORTD, sendPin);                // Send HIGH
                                            //
    while (!tst_bit(PIND, receivePin[MUX])) // Enquanto Receive está LOW
      total++;                              // Incrementa o valor
                                            //
    set_bit(DDRD, receivePin[MUX]);         // Receive OUTPUT
    set_bit(PORTD, receivePin[MUX]);        // Receive HIGH
    _delay_us(3);                           // Espera 10ms
    clr_bit(DDRD, receivePin[MUX]);         // Receive INPUT
    clr_bit(PORTD, receivePin[MUX]);        // Receive PULLUP desligados
    clr_bit(PORTD, sendPin);                // Send LOW
                                            //
    while (tst_bit(PIND, receivePin[MUX]))  // Enquanto Receive está HIGH
      total++;                              // Incrementa o valo
  }

  return total;
}
void ReadAll()
{
  for (uint8_t casa = 0; casa < 64; casa++)
  {                                                 //  Liga o MUX daquela casa
    if (!tst_bit(casa, 5) && !tst_bit(casa, 4))     // "Abrindo" binario, 00 liga a porta 4
      PORTC = 0b11100000 + casa % 16;               // Salvando no PORTC a casa escolhida
    else if (!tst_bit(casa, 5) && tst_bit(casa, 4)) // "Abrindo" binario, 01 liga a porta 5
      PORTC = 0b11010000 + casa % 16;               // Salvando no PORTC a casa escolhida
    else if (tst_bit(casa, 5) && !tst_bit(casa, 4)) // "Abrindo" binario, 10 liga a porta 6
      PORTC = 0b10110000 + casa % 16;               // Salvando no PORTC a casa escolhida
    else if (tst_bit(casa, 5) && tst_bit(casa, 4))  // "Abrindo" binario, 11 liga a porta 7
      PORTC = 0b01110000 + casa % 16;               // Salvando no PORTC a casa escolhida

    // Faz a leitura e salva uma media dos ultimos valores
    value[casa] += (ReadCapacitiveSensor(10, casa / 16) - value[casa]) / 1.50; // Somando parte da leitura atual na variavel que guarda o valor, para deixar a curva mais suave
    media[casa] += (value[casa] - media[casa]) / divisionValue;                // Media dos ultimos valores lidos, para deixar as leituras lineares
  }
  trigger(); // Testa para ver se algum sensor pode ser considerado ativado, se sim, escolhe o maior
}

// Funções para identificar movimentação
void getMove()
{
  // Pega o movimento e converte para enviar
  // Os numeros 49 e 97 utilizados adiante são devido a posição do numero '1' e da letra 'a' na tabela ASCII
  // E as multiplicações por 8 são devido as 8 casas por linha no tabuleiro

  while (Rselected == 100 || Rselected == fCasa)            // Espera até que seja lido uma casa
    ReadAll();                                              // Le os sensores
                                                            // Salva a cor do led antes do movimento
  uint8_t start, startR, startG, startB;                    // Declara variaveis para salvar a cor ca casa escolhida
  start = Rselected;                                        // Para evitar de uma peça ser movida para a casa que ja está
  if (start / 8 % 2)                                        // Se se for uma linha impar
  {                                                         //
    int AUX = (63 - start) * 3;                             // Conversão
    startG = Leds[AUX];                                     // Salvando a cor do led na sua variavel
    startR = Leds[AUX + 1];                                 //
    startB = Leds[AUX + 2];                                 //
  }                                                         //
  else                                                      // Se for uma linha par
  {                                                         //
    int AUX = (64 - ((start / 8 + 1) * 8) + start % 8) * 3; // Conversão
    startG = Leds[AUX];                                     // Salvando a cor do led na sua variavel
    startR = Leds[AUX + 1];                                 //
    startB = Leds[AUX + 2];                                 //
  }                                                         //
                                                            //
  Smove[0] = Rselected % 8 + 97;                            // Converte do numero da casa para coordenadas do xadrez
  Smove[1] = Rselected / 8 + 49;                            // E salva em char
  corCasa(Rselected, 0, 255, 0);                            // Acende o led da casa
  LedShow();                                                // Grava a cor no led
                                                            //
  while (Rselected == start)                                // Verifica a retirada a peça do lugar
    ReadAll();                                              // Le os sensores
                                                            //
  while (Rselected == 100)                                  // Espera até que seja lido uma casa
    ReadAll();                                              // Le os sensores
  Smove[2] = Rselected % 8 + 97;                            // Converte do numero da casa para coordenadas do xadrez
  Smove[3] = Rselected / 8 + 49;                            // E salva em char
  if (pColor)                                               // Se for brancas
    corCasa(Rselected, 150, 0, 0);                          // Acende a cor vermelha na nova casa da peça
  else                                                      // Senão
    corCasa(Rselected, 0, 0, 150);                          // Acende a cor azul
                                                            //
  if (Rselected == start)                                   // Se a casa de inicio e a de destino da peça for a mesma, o usuario
  {                                                         // cancelou o movimento e ele deve ser repetido
    corCasa(start, startR, startG, startB);                 // Acende a antiga casa da peça
    LedShow();                                              // Grava a cor no led
    while (Rselected == start)                              // Espera enquanto a peça esta sendo manipulada
      ReadAll();                                            // Le os sensores
    getMove();                                              // Retorna para o inicio dessa função
  }                                                         //
  else                                                      // Senão
  {                                                         //
    corCasa(start, 0, 0, 0);                                // Apaga a antiga casa da peça
    LedShow();                                              // Grava a cor no led
  }
}
void gotMoved()
{

  sCasa = (Rmove[1] - 49) * 8 + Rmove[0] - 97; // Converte coordenada do xadrez recebida usando Serial para Numero da casa
  fCasa = (Rmove[3] - 49) * 8 + Rmove[2] - 97; // E salva numa variavel char

  // Confere se a casa recebida foi movida
  for (int i = 0; i < 10; i++) // Calibragem rapida
    ReadAll();                 // Le os sensores
  while (Rselected != sCasa)   // Espera até que seja lida a casa requerida
    ReadAll();                 // Le os sensores
  corCasa(sCasa, 0, 0, 0);     // Apaga o led da casa
  LedShow();                   // Grava os leds
                               //
  while (Rselected != fCasa)   // Espera até que seja lida a casa requerida
    ReadAll();                 // Le os sensores
  if (pColor)                  // Se for brancas
    corCasa(fCasa, 0, 0, 150); // Acende a cor vermelha na nova casa da peça
  else                         // Senão
    corCasa(fCasa, 150, 0, 0); // Acende a cor azul
  LedShow();                   // Grava a cor no led
}

// DELETAR DEPOIS - apenas para testes
//  Serve para acender o led que foi identificado movimento
void Sempre()
{
  // Calibragem();
  while (true)
  {
    ReadAll();
    for (int i = 0; i < 192; i++)
      Leds[i] = 0;
    if (Rselected != 100)
    {
      corCasa(Rselected, 0, 0, 255);
      LedShow();
    }
    else
      LedShow();
  }
}

int main(void)
{
  DDRB = 0b00100000;           // led OUTPUT
  PORTB = 0b00000000;          // led LOW
                               //
  DDRC = 0b11111111;           // S0..S3 e EN0..EN3 output
  PORTC = 0b00000000;          // S0..S3 e EN0..EN3 low
                               //
  DDRD = 0b10000000;           // send output, receives input
  PORTD = 0b00000000;          // send pin low
                               //
  SerialBegin(9600);           // Inicia o Serial, com baud rate de 9600
  LedShow();                   // Reseta os leds
  for (int i = 0; i < 30; i++) // Calibra a variavel media e value
    ReadAll();                 // Le os sensores

  while (true)
  {
    // Sempre();
    Data = SerialRead(); // Fica esperando receber algum dado

    if (Data == 'M')
    {                                              // Recebendo um movimento executado pela engine no PC
      for (int i = 0; i < 4; i++)                  // Le 4 vezes
        Rmove[i] = SerialRead();                   // O movimento chegando
                                                   //
      sCasa = (Rmove[1] - 49) * 8 + Rmove[0] - 97; // Converte do numero da casa para coordenadas do xadrez
      fCasa = (Rmove[3] - 49) * 8 + Rmove[2] - 97; // E salva em char
      corCasa(sCasa, 0, 255, 0);                   // Acende o led da casa
      corCasa(fCasa, 0, 255, 0);                   // Acende o led da casa
      LedShow();                                   // Grava a cor no led
                                                   //
      gotMoved();                                  // Função para verificar se o movimento requisitado pelo PC foi executado
      getMove();                                   // Função para pegar um novo movimento
                                                   //
      for (uint8_t i = 0; i < 4; i++)              // Repete por 4 vezes
        SerialWrite(Smove[i]);                     // O envio dos 4 char usando Serial
    }
    else if (Data == 'N')
    {                                   // Recebendo o comando de nova partida
      for (int i = 0; i < 74; i++)      // Religando os leds
        switch (i / 16)                 // Convertendo em conjuntos de 16 leds
        {                               //
        case 0:                         // Se for entre os leds 0 e 15
          corCasa(i, 150, 0, 0);        // Liga eles vermelhos
          break;                        //
        case 3:                         // Se for entre o 48 e o 63
          corCasa(i, 0, 0, 150);        // Liga azul
          break;                        //
        case 4:                         // Se for os leds de avaliação
          if (i < 69)                   // Se for do lado branco
            corCasa(i, 150, 0, 0);      // Liga vermelho
          else                          // Senão é do lado preto
            corCasa(i, 0, 0, 150);      // Laga azul
          break;                        //
        default:                        // Para os outros leds
          corCasa(i, 0, 0, 0);          // São desligados
          break;                        //
        }                               //
      LedShow();                        // Grava os leds
                                        //
      if (SerialRead() == 'w')          // Caso o player seja brancas
      {                                 //
        pColor = true;                  // Coloca na variavel para lembrar disso
        getMove();                      // Coleta o movimento
        for (uint8_t i = 0; i < 4; i++) // Para os 4 bytes
          SerialWrite(Smove[i]);        // Envia o movimento usando Serial
      }                                 //
      else                              // Se for preto
        pColor = false;                 // Apenas coloca na variavel
    }
    else if (Data == 'E')
    {
      // Recebendo dados do serial
      char sig = SerialRead();    // Recebe qual lado está melhor
      char Eval[4];               // Variavel para guardar as infos recebidas, sendo composto por 3 numeros e um ponto de decimal
      for (int i = 0; i < 4; i++) // Repete por 4 vezes
        Eval[i] = SerialRead();   // Recebendo o numero da avaliação
      eval = atoi(Eval);          // Converte os 4 chars em um numero - atoi presente na lib <stdlib.h>
                                  //
      // Convertendo entre 0 e 10
      if (sig == '+')        // Se for positivo
        eval = 5 + eval / 2; // Soma os leds
      else if (sig == '-')   // Se for negativo
        eval = 5 - eval / 2; // Subtrai os leds

      // Ligando os Leds com a cor correspondente da avaliação
      for (int i = 0; i < 10; i++)    // Repete para os dez leds de avaliação
        if (i < eval)                 // Se estiver antes do valor de corte (eval)
          corCasa(i + 64, 150, 0, 0); // Liga vermelho
        else                          // Senão
          corCasa(i + 64, 0, 0, 150); // Liga azul
      LedShow();
    }
  }
}
