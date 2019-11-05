/**
 * GPS_Carro.cpp       v0.0        22-10-2019 
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
 * Essa breve explicacao tenta esclarecer como funciona a 
 * decodificacao dos valores a serem armazenados.
 * 
 * Todos os valores são inicialmente armazenados em uma string e então, o endereço dessa string
 * é passado como parâmetro para que se fosse extrair os dados dessa string e armazenar na estrutura
 * que tem o endereço passado como parâmetro.
 *
 * A primeira análise é feita pela função "parse", verificando se o protocolo obtido é o "$GPRMC". Se
 * for verdade, então os dados são extraídos da string para serem armazenados; caso contrário, os dados
 * são "descartados".
 *
 * Um reset é dado na estrutura para excluir possíveis valores anteriormente armazenados, para que não
 * influenciem no cálculo dos novos valores.
 *
 * Após isso, cada um das funções abaixo saí a procura de vírgulas ',', pois elas dividem cada um dos atributos
 * fornecidos pelo GPS. Os dados são armazenados até que se encontre uma vírgula, e a posição dessa vírgula
 * é salva para que na busca do próximo dado, não precise se percorrer uma parte do vetor já percorrida.
 * 
 * Por fim, algumas conversões como transformação da latitude e longitude em graus com adição do devido sinal 
 * negativo ou positivo (- ou +), e também conversão da unidade de velocidade de knots para km/h são realizadas.
 *----------------------------------------------------------------------------------------------------------------------
 */ 

#include "GPS_Carro.h"

void parse (char *cmd, int n, dataGPS *data) {
    if (strncmp (cmd,"GPRMC", 5) == 0) {
        dataCatch (cmd, data);       
    }
    return;
}


// Fun��o para reiniciar as variaveis numericas
void reset (dataGPS *data) {
    data->time = 0;
    data->latitude = 0;
    data->longitude = 0;
    data->speed = 0;
    data->course = 0;
    data->magnetcVariationValue = 0;
}


// Fun��o para pegar o protocolo
int getProtocol (char *input, int pivot, dataGPS *data) {
    int i;
    // Executa at� encontrar uma virgula
    for (i = 0; (int)input[pivot + i] != ',' ; i++) //i < 6
    {
        data->protocol[i] = input[pivot + i];  
    }
   
    // Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para calcular uma potencia de 10
int potencia (int val) {
    int i, j;
    j = 1;
    for (i = 0; i < val; i++) {
        j *= 10;
    }

    return j;
}

// Fun��o para pegar o tempo do gps
int getTime (char *input, int pivot, dataGPS *data) {
    int i, j;
    // Executa at� encontrar uma virgula
    for (i = 0; (int)input[pivot + i] != ',' ; i++) { // i < 9 
        // Pega os valores diferentes de '.'
        if ( (double)input[pivot + i] != '.') {
            j = potencia (i);
            data->time += ( (double)input[pivot + i] - '0')* (100000/j);
        }
    }
     //Retorna a posi��o do ultimo dado pego
     data->time -= 30000.0;
    return pivot + i - 1;
}

// Fun��o para pegar o bit de validade do gps
int getValid (char *input, int pivot, dataGPS *data) {
    data->valid = input[pivot];
   
    return pivot;
}

// Fun��o para pegar a latitude do gps
int getLatitude (char *input, int pivot, dataGPS *data) {
    int i = 0, j;
    double k = 1000.00;
    // Executa at� encontrar uma virgula
    for(i = 0; (int)input[pivot + i] != ',' ; i++) {
        // Pega os valores diferentes de '.'
        if( (double)input[pivot + i] != '.') {
            j = potencia (i);
            data->latitude += ( (double)input[pivot + i] - '0')* (k/j);
        }
        else {
            // Ajusta a potencia
            k *= 10;
        }
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar a dire��o da latitude do gps
int getDirection1 (char *input, int pivot, dataGPS *data)
{
    data->direction1 = input[pivot];
   
    return pivot;
}

// Fun��o para pegar a longitude do gps
int getLongitude (char *input, int pivot, dataGPS *data)
{
    int i = 0, j;
    double k = 10000.00;
    // Executa at� encontrar uma virgula
    for(i = 0; (int)input[pivot + i] != ',' ; i++) { // i < 10
        // Pega os valores diferentes de '.'
        if((double)input[pivot + i] != '.') {
            j = potencia (i);
            data->longitude += ( (double)input[pivot + i] - '0')* (k/j);
        }
        else {
            // Ajusta a potencia
            k *= 10;
        }
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar a dire��o da longitude do gps
int getDirection2 (char *input, int pivot, dataGPS *data)
{
    data->direction2 = input[pivot];
   
    return pivot;
}

// Fun��o para pegar a velocidade do gps
int getSpeed (char *input, int pivot, dataGPS *data)
{
    int i = 0, j;
    double k = 100.00;
    // Executa at� encontrar uma virgula
    for(i = 0; (int) input[pivot + i] != ',' ; i++) {
        // Pega os valores diferentes de '.'
        if( (double) input[pivot + i] != '.') {
            j = potencia (i);
            data->speed += ( (double)input[pivot + i] - '0')* (k/j);
        }
        else {
            // Ajusta a potencia
            k *= 10;
        }
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar a dire��o do deslocamento do gps
int getCourse(char *input, int pivot, dataGPS *data) {
    int i = 0, j;
    double k = 100.00;
    // Executa at� encontrar uma virgula
    for(i = 0; (int)input[pivot + i] != ',' ; i++) {
        // Pega os valores diferentes de '.'
        if( (double)input[pivot + i] != '.') {
            j = potencia(i);
            data->course += ((double)input[pivot + i] - '0')* (k/j);
        }
        else {
            // Ajusta a potencia
            k *= 10;
        }
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar a data do gps
int getDate (char *input, int pivot, dataGPS *data) {
    int i;
    // Executa at� encontrar uma virgula
    for(i = 0; (int) input[pivot + i] != ',' ; i++) { //i < 6 
        data->date[i] = input[pivot + i];  
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar a varia��o magn�tica do gps
int getMagnetcVariationValue (char *input, int pivot, dataGPS *data) {
    int i = 0, j;
    double k = 100.00;
    // Executa at� encontrar uma virgula
    for(i = 0; (int)input[pivot + i] != ',' ; i++) {
        // Pega os valores diferentes de '.'
        if( (double)input[pivot + i] != '.') {
            j = potencia(i);
            data->magnetcVariationValue += ((double)input[pivot + i] - '0')* (k/j);
        }
        else {
            // Ajusta a potencia
            k *= 10;
        }
    }
   
    //Retorna a posi��o do ultimo dado pego
    return pivot + i - 1;
}

// Fun��o para pegar o modo do gps
int getMode (char *input, int pivot, dataGPS *data) {
    data->mode = input[pivot];
    if( (int)data->mode == ',')
        return pivot - 1;
    else
        return pivot;
}

// Fun��o para pegar o checksum do gps
void getChecksum (char *input, int pivot, dataGPS *data)
{
    int i;
    for(i = 0; i < 4 ; i++) { //i < 6 
        data->checksum[i] = input[pivot + i];  
    }
}


// Fun��o para pegar todos os dados do gps
void dataCatch (char  *mensagem, dataGPS *data) {   
    int i = 0, j = 0;
    reset (data);
    while(j < 12) {
        // A cada virgula o j � incrementado
        if( (int)mensagem[i] == ',') {
            j++;
            i++;
        }
        // Escolhendo que dado ser� pego
        switch (j) {
            case 0:
                //fun��o getProtocol
                i = getProtocol (mensagem, i, data);
                break;
            case 1:
                //fun��o getTime
                i = getTime (mensagem, i, data);
                break;
            case 2:
                //fun��o getValid
                i = getValid (mensagem, i, data);
                break;
            case 3:
                //fun��o getLatitude
                i = getLatitude (mensagem, i, data);
                break;
            case 4:
                //fun��o getDirection1
                i = getDirection1 (mensagem, i, data);
                break;
            case 5:
                //fun��o getLongitude
                i = getLongitude (mensagem, i, data);
                break;
            case 6:
                //fun��o getDirection2
                i = getDirection2 (mensagem, i, data);
                break;
            case 7:
                //fun��o getSpeed
                i = getSpeed (mensagem, i, data);
                break;
            case 8:
                //fun��o getCourse
                i = getCourse (mensagem, i, data);
                break;
            case 9:
                //fun��o getDate
                i = getDate (mensagem, i, data);
                break;
            case 10:
                //fun��o getMagnetcVariationValue
                i = getMagnetcVariationValue (mensagem, i, data);
                break;
            case 11:
                //fun��o getMagnetcVariationIndicator
                i = getMode (mensagem, i, data);
                break;
            case 12:
                //fun��o getChecksum
                getChecksum (mensagem, i, data);
                break;
        }
       
        i++;
    }
    transformaCoordenada (data);
    transformaSpeed (data);
    
}


// Fun��o que ajusto o formato das coordenadas
void transformaCoordenada (dataGPS *data)
{
    double num1, num2;
    double aux, aux2;

    num1 = data->latitude;
    num2 = data->longitude;
   
    num1 = num1/100;
    aux = num1;
    aux = aux - (int)aux;
    aux = aux*100;
    aux = aux/60;
    num1 = (int)num1;
   
    num2 = num2/100;
    aux2 = num2;
    aux2 = aux2 - (int)aux2;
    aux2 = aux2*100;
    aux2 = aux2/60;
    num2 = (int)num2;
   
    data->latitude =  num1 + aux;
    data->longitude =  num2 + aux2;
   
    if ( (int)data->direction1 == 'S')
        data->latitude *= -1;
   
    if ( (int)data->direction2 == 'W')
        data->longitude *= -1;
}

// Fun��o que passa a velocidade para km/h
void transformaSpeed (dataGPS *data) {
    data->speed = data->speed * 1.853;
}
