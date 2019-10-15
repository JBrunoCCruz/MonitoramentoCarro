/**
 * payloadCarro.h       v0.0        08-10-2019 
 *
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
 * No projeto, tinha-se a ideia de adicionar os seguintes
 * dados posteriormente:
 * 
 * Velocidade - 1 float
 * Rotacao - 1 float
 * Temperatura da agua - 1 float
 * Posicao da borboleta	- 1 float
 * Pressao do coletor de admissao - 1 float  
 * GPS - ? bytes
 *----------------------------------------------------------------------------------------------------------------------
 */


/**
 *----------------------------------------------------------------------------------------------------------------------
 * INCLUSÃO DE BIBLIOTECAS E DIRETIVAS 'DEFINE'
 *----------------------------------------------------------------------------------------------------------------------
 */

#ifndef _PAYLOAD_CARRO_H_
#define _PAYLOAD_CARRO_H_

#include "mbed.h"
 
#define CONSTANTE_MULTIPLICADOR 1000
#define QUANTIDADE_DE_DADOS 19

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Classe de implementação do buffer de envio na rede LoRa
 * O buffer pode ter tamanho de no máximo 64 bytes (isso depende
 * das configurações LoRa, mas no meu caso é 64 bytes).
 * Se o seu buffer é maior ou menor, essa quantidade pode (ou deve)
 * ser alterada nessa classe também em: QUANTIDADE_DE_DADOS.
 *----------------------------------------------------------------------------------------------------------------------
 */
class PayLoadCarro {
	public:
        //Outros dados podem ser adicionados (ou excluídos), contanto que não ultrapasse um total de 64 bytes		

        /**
        *----------------------------------------------------------------------------------------------------------------------
        * Adiciona os dados de um acelerometro ao buffer de envio
        *
        * Cada um dos eixos é passado como parametro para função 
        * A precisão é de duas casas decimais, o programador pode 
        * ficar a vontade para aumentar essa quantidade tomando 
        * cuidado para não exceder o tamanho do buffer (64 bytes).    
        *
        * @param x                     aceleração 1
        * @param y                     aceleração 2
        * @param z                     aceleração 3
        *
        * @return                      Não retorna nada, apenas
        *                              adiciona os valores de aceleração
        *                              ao buffer de envio ('dados[]')
        *----------------------------------------------------------------------------------------------------------------------
        */
		uint8_t addAccelerometer (float x, float y, float z);

        /**
        *----------------------------------------------------------------------------------------------------------------------
        * Adiciona os dados de um sensor de temperatura ao buffer
        *
        * A temperatura pode ser passada em qualquer escala
        * A precisão é de duas casas decimais, o programador pode 
        * ficar a vontade para aumentar essa quantidade tomando 
        * cuidado para não exceder o tamanho do buffer (64 bytes).
        *
        * @param temperatura           o valor da temperatura a ser adicionado    
        *
        * @return                      Não retorna nada, apenas
        *                              adiciona o valor de temperatura
        *                              ao buffer de envio ('dados[]')
        *----------------------------------------------------------------------------------------------------------------------
        */
		uint8_t addTemperature (float temperatura);
        
        /**
        *----------------------------------------------------------------------------------------------------------------------
        * Adiciona os dados de um calenderio e relogio ao buffer
        *
        * Os dados de um calendario e de relogio são passados como parametros
        * para serem adicionados ao buffer de nvio.
        *        
        * @param dia                     Dia do ano [Dom..Sab]
        * @param mes                     Mes do ano [Jan..Dez]
        * @param ano                     Ano        [2000..2099]
        * @param hora                    Formato escolhido de acordo com a necessidade [12h - 24h]
        * @param minuto                  Minutos    [0..59]
        * @param segundo                 Segundos   [0..59]
        *
        * @return                        Não retorna nada, apenas
        *                                adiciona os valores de tempo
        *                                ao buffer de envio ('dados[]')
        *----------------------------------------------------------------------------------------------------------------------
        */
		uint8_t addRTC (uint8_t dia, uint8_t mes, uint16_t ano, uint8_t hora, uint8_t minuto, uint8_t segundo);		

    public:
        /**
        *----------------------------------------------------------------------------------------------------------------------
        * Distruibuição dos dados no buffer
        *
        * acelerometro - bytes 0 a 8
        * temperatura  - bytes 9 a 11
        * relogio      - bytes 12 a 18
        *----------------------------------------------------------------------------------------------------------------------
        */
        uint8_t dados[QUANTIDADE_DE_DADOS];
        uint8_t tamanho = QUANTIDADE_DE_DADOS;        
};
  
#endif /*_PAYLOAD_CARRO_H_*/