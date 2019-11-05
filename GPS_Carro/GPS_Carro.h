/**
 * GPS_Carro.h       v0.0        22-10-2019 
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
 
#ifndef _GPS_CARRO_H_
#define _GPS_CARRO_H_
 
#include "mbed.h"

/**
 *----------------------------------------------------------------------------------------------------------------------
 * @brief   Definição da estrutura para armazenar os dados oriundos do GPS
 *          O protocolo escolhido para aquisição dos dados foi: $GPRMC
 *          Os protocolos restantes podem ser vistos no seguinte endereço:
 *              https://os.mbed.com/users/edodm85/notebook/gps-u-blox-neo-6m/
 *
 * @var protocol                      protocolo (GPGGA, GPGSA, GPGLL, GPRMC)
 * @var time                          tempo UTC (Fuso horário de referência (-3° em relação ao Brasil))  
 * @var valid                         validade dos dados fornecidos pelo GPS (Semelhante ao CheckSum)
 * @var latitude                      latitude
 * @var getDirection1                 direção da latitude (Norte ou Sul)
 * @var longitude                     longitude absoluta
 * @var direction2                    direção da longitude (Leste ou Oeste)
 * @var speed                         velocidae fornecida em knots (equivalente a 1.853km/h)
 * @var course                        angulo de movimentacao
 * @var data                          data no formato: ddmmaa
 * @var magnetcVariationValue         
 * @var magnetcVariationIndicator
 * @var mode
 * @var checksum                      soma de verificação (usado para checar a validade dos dados)
 *----------------------------------------------------------------------------------------------------------------------
 */
typedef struct {
    char protocol[8];
    double time;
    char valid;
    double latitude;
    char direction1;
    double longitude;
    char direction2;
    double speed;
    double course;
    char date[7];
    double magnetcVariationValue;
    char magnetcVariationIndicator;
    char mode;
    char checksum[5];
} dataGPS;

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Protótipo das funções
 *----------------------------------------------------------------------------------------------------------------------
 */

/**
 *----------------------------------------------------------------------------------------------------------------------
 * @brief Verifica se o a string passada corresponde a "$GPRMC". Se for igual,
 *        é chamado um procedimento para a aquisição dos dados; caso contrário,
 *        a função apenas retorna para o ponto onde foi chamada.
 *
 * @param cmd           ponteiro para a string de dados obtida do GPS   
 * @param i             tamanho da string obtida (cmd)
 * @param data          ponteiro para a struct ao qual os dados serão armazenados    
 *
 * @return                      Não retorna nada, apenas
 *                              adiciona os valores lidos a 
 *                              estrutura apontada pelo ponteiro data.
 *----------------------------------------------------------------------------------------------------------------------
 */
void parse (char *cmd, int n, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Procedimento utilizado para invocar cada um das funções que irão de fato
*        obter os dados fornecidos pelo GPS. São utilizadas duas variáveis 'i' e 'j'
*        como uma maneira para se obter cada um dos dados.
*
* @param mensagem      ponteiro para a string de dados obtida do GPS    
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              invoca cada um dos procedimentos
*                              para a aquisição dos dados.
*----------------------------------------------------------------------------------------------------------------------
*/
void dataCatch (char *mensagem, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Reseta a estrutura para a aquisição dos dados mais atualizados
*
* @param data          ponteiro para a struct ao qual os dados serão resetados  
*
* @return                      Não retorna nada, apenas
*                              reseta a estrutura apontada 
*                              pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
void reset (dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o protocolo fornecido pelo GPS (nesse programa em específico, apenas $GPRMC)
*        armazenado no vetor apontado por input
*
* @param input         ponteiro para a string de dados obtida do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o protocolo obtido de 'input' 
*                              a estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getProtocol (char *input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o tempo armazenado no vetor apontado por input
*
* @param input         ponteiro para a string de dados obtida do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona os valores de tempo a  
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getTime (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o caractere que indica a validade dos dados (V ou A)
*
* @param input         ponteiro para a string de dados obtida do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o caractere de validade a
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getValid (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a latitude armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o valor da latitude a
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getLatitude (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a direção da latitude (N ou S) armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados     
*
* @return                      Não retorna nada, apenas
*                              adiciona o valor de direção a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getDirection1 (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a longitude armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o valore da longitude a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getLongitude (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a direção da latitude (L ou O) armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o valor de direção a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getDirection2 (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a velocidade armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados   
*
* @return                      Não retorna nada, apenas
*                              adiciona o valor de velocidade a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getSpeed (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o valor do curso (angulo de movimentação) armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados   
*
* @return                      Não retorna nada, apenas
*                              adiciona o valor de curso a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getCourse (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a data armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados   
*
* @return                      Não retorna nada, apenas
*                              adiciona os valores de data a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getDate (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a variancia magnetica armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona os valores de variancia magnetica a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getMagnetcVariationValue (char input,int pivot,  dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o indicador de variancia magnetica armazenado no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o indicador de variancia magnetica a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getMagnetcVariationIndicator (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega o modo armazenado no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados     
*
* @return                      Não retorna nada, apenas
*                              adiciona o modo a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
int getMode (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Pega a soma de verificação armazenada no vetor apontado por input
*
* @param input         ponteiro para a string de dados obitda do GPS   
* @param pivot         utilizado para indicar qual parâmetro está sendo obtido
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona os valores de CheckSum a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
void getChecksum (char input, int pivot, dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Adiciona o sinal a coordenada com base na direção, uma vez que a coordenada é dada em valores absolutos
*
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              adiciona o sinal na longitude e latitude a 
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
void transformaCoordenada (dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Transforma a velocidade de knots pra km/h
*
* @param data          ponteiro para a struct ao qual os dados serão armazenados    
*
* @return                      Não retorna nada, apenas
*                              muda a unidade da velocidade na
*                              estrutura apontada pelo ponteiro data.
*----------------------------------------------------------------------------------------------------------------------
*/
void transformaSpeed (dataGPS *data);

/**
*----------------------------------------------------------------------------------------------------------------------
* @brief Função para calcular potencia de 10
*
* @param val         expoente do 10    
*
* @return                      retorna a potencia de 10 
*                              elevado a 'val'.
*----------------------------------------------------------------------------------------------------------------------
*/
int potencia (int val);
 
#endif /*_GPS_CARRO_H_*/