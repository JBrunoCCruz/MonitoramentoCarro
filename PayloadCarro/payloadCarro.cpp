/**
 * payloadCarro.cpp       v0.0        08-10-2019 
 *
 * Orientador: Elias Teodoro da Silva Junior
 * Autores: Felipe Moura de Castro e Joao Bruno Costa Cruz,
 * Instituto Federal de Educação, Ciência e Tecnologia do Ceará (IFCE) - Campus Fortaleza
 *
 * @Opensource
 * Este código-fonte pode ser utilizado, copiado, estudado, modificado e redistribuído sem restrições.
 *
 * Copyright (c) 2019, Felipe Moura de Castro e Joao Bruno Costa Cruz.
 *
 */

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Essa breve explicacao tenta esclarecer como funciona a codificacao
 * e a decodificacao dos valores a serem enviados.
 * Normalmente os valores com os quais trabalhamoss são numeros reais,
 * ou seja, negativos, positivos e zero.
 * A funcao de "envia" da biblioteca 'lora' ('send'), trabalha recebendo
 * inteiros sem sinais (unsigned int) para enviar os dados pela rede.
 * Dessa forma, e necessario encontrar uma maneira de enviar valores
 * negativos; a solucao encontrada foi a seguinte:
 * Divide-se o numero pelo seu modulo para encontrar o fator somador
 * que será -1 se o numero for negativo, e 1 caso seja positivo. Apos
 * isso, soma-se com 46; se o numero for negativo, o resultado da soma
 * sera 45 que equivale ao sinal '-' em caractere, e entao enviamos pela
 * rede um bit para informar o sinal antes de enviar o valor
 *
 * Ja que pode-se apenas enviar bytes pela rede, os numero sao divididos
 * em duas partes: 1 byte para a parte inteira e 1 byte para a parte 
 * fracionaria. Em seguida envia-se pela rede e o valor é reconstruido
 * do outro lado.
 *
 * Ex: -2.5
 * -2/5 / |-2.5| = -1 (fator somador)
 * 46 + (-1) = 45 (numero é negativo)
 * parte inteira = abs (-2.5) = 2
 * parte fracionaria = (abs (-2.5) - 2 ) * 100 = 50
 * numero =   -         2       50
 *          byte 0 | byte 1 | byte 2
 *          sinal  | inteira| fracionaria
 *----------------------------------------------------------------------------------------------------------------------
 */

#include "payloadCarro.h"

uint8_t PayLoadCarro::addAccelerometer (float x, float y, float z) {
	dados[0] = 46 + (x / fabs (x) ); //sinal, '-' em Hexadecimal = 0x2D (45)
    dados[1] = fabs (x);
	dados[2] = (fabs (x) - dados[1] ) * 100;
    dados[3] = 46 + (y / fabs (y) ); //sinal, '-' em Hexadecimal = 0x2D (45)
	dados[4] = fabs (y);
	dados[5] = (fabs (y) - dados[4] ) * 100;
    dados[6] = 46 + (z / fabs (z) ); //sinal, '-' em Hexadecimal = 0x2D (45)
	dados[7] = fabs (z);
	dados[8] = (fabs (z) - dados[7] ) * 100;
	return 0; 
}

uint8_t PayLoadCarro::addTemperature (float temperatura) {
    dados[9] = 46 + (temperatura / fabs (temperatura) ); //sinal, '-' em Hexadecimal = 0x2D (45)
    dados[10] = fabs (temperatura);
	dados[11] = (fabs (temperatura) - dados[10] ) * 100;
    return 0; 
}

uint8_t PayLoadCarro::addRTC (uint8_t dia, uint8_t mes, uint16_t ano, uint8_t hora, uint8_t minuto, uint8_t segundo) {
    dados[12] = ano / 100;
    dados[13] = (ano - (dados[12] * 100) ); //Divide o ano em duas partes. Ex: 2019 -> 20 | 19
    dados[14] = mes;
    dados[15] = dia;
    dados[16] = hora;
    dados[17] = minuto;
    dados[18] = segundo;
    return 0; 
}

uint8_t PayLoadCarro::addGPSData (uint8_t dia, uint8_t mes, uint16_t ano, uint8_t hora, uint8_t minuto, uint8_t segundo) {
    dados[12] = ano / 100;
    dados[13] = (ano - (dados[12] * 100) ); //Divide o ano em duas partes. Ex: 2019 -> 20 | 19
    dados[14] = mes;
    dados[15] = dia;
    dados[16] = hora;
    dados[17] = minuto;
    dados[18] = segundo;
    return 0; 
}

uint8_t PayLoadCarro::addGPS (double Latitude, double Longitude, double Velocidade) {
	dados[19] = 46 + (Latitude / fabs (Latitude) ); //sinal, '-' em Hexadecimal = 0x2D (45)
    dados[20] = fabs (Latitude);
	dados[21] = (fabs (Latitude) - dados[20] ) * 100;
    dados[22] = 46 + (Longitude / fabs (Longitude) ); //sinal, '-' em Hexadecimal = 0x2D (45)
	dados[23] = fabs (Longitude);
	dados[24] = (fabs (Longitude) - dados[23] ) * 100;
    dados[25] = 46 + (Velocidade / fabs (Velocidade) ); //sinal, '-' em Hexadecimal = 0x2D (45)
	dados[26] = fabs (Velocidade);
	dados[27] = (fabs (Velocidade) - dados[26] ) * 100;
	return 0; 
}

