
  Essa breve explicacao tenta esclarecer como funciona a 
  decodificacao dos valores a serem armazenados.
  
  Todos os valores são inicialmente armazenados em uma string e então, o endereço dessa string
  é passado como parâmetro para que se fosse extrair os dados dessa string e armazenar na estrutura
  que tem o endereço passado como parâmetro.
 
  A primeira análise é feita pela função "parse", verificando se o protocolo obtido é o "$GPRMC". Se
  for verdade, então os dados são extraídos da string para serem armazenados; caso contrário, os dados
  são "descartados".
 
  Um reset é dado na estrutura para excluir possíveis valores anteriormente armazenados, para que não
  influenciem no cálculo dos novos valores.
 
  Após isso, cada um das funções abaixo saí a procura de vírgulas ',', pois elas dividem cada um dos atributos
  fornecidos pelo GPS. Os dados são armazenados até que se encontre uma vírgula, e a posição dessa vírgula
  é salva para que na busca do próximo dado, não precise se percorrer uma parte do vetor já percorrida.
  
  Por fim, algumas conversões como transformação da latitude e longitude em graus com adição do devido sinal 
  negativo ou positivo (- ou +), e também conversão da unidade de velocidade de knots para km/h são realizadas.
  
  ## Obtendo os dados do GPS
  
  A obtenção dos dados é feito por uma simples comunicação UART. O GPS fica a todo momento (mesmo não estando conectado a algum satélite) enviando os dados via comunicação serial.
  
  ## Links - protocolos
  
  <p>A forma como esses dados enviados seguem alguns protocolos. Esses protocolos podem ser vistos no link a seguir:</p>
  
[GPS protoclos](https://os.mbed.com/users/edodm85/notebook/gps-u-blox-neo-6m)
  
  ## Outros Links - Exemplos 
  
  <p>Estes outros links podem ser utilizados como exemplos a serem seguidos, dando uma explanação inicial ao leitor</p>

  [GPS Arduino](https://blog.eletrogate.com/gps-neo-6m-com-arduino-aprenda-usar/) | 
  [GPS Arduino - Link 2](https://portal.vidadesilicio.com.br/modulo-gps-neo-6m/) | 
  [GPS Mbed OS - Exemplo de programa](https://os.mbed.com/users/edodm85/code/GPS_U-blox_NEO-6M_Test_Code/file/acd907fbcbae/main.cpp/)
