 ---------------------------- WI FI Connection ---------------------------------------

Overview
Conectarse al internet por primera vez del lugar, es lo primero que hace el dispositivo

Pseudocódigo
1. Invoca la librería de Arduino para conectarse a Wi Fi usando las credenciales declaradas al principio
2. Espera hasta que se acabe el wifi timeout o hasta que el estatus de la red sea "conectado"

	2.1 Si el timeout vence se pone el dispositivo en modo NO_WIFI_MODE

	
 ------------------------------ FIREBASE Connection -------------------------------------------

Overview
Conectarse a la herramienta de Google que nos informa el status de las encuestas que llenan los usuarios

Pseudocódigo
0. Se verifica que no se esté en NO_WIFI_MODE, si no se salta todo
1. Se asigna el url y la llave del servidor a conectarse
2. Se usa esa asignación de url y llave para autenticarse con el servidor
3. Se usa una función de la librería para informar que se va a iniciar el intercambio de datos

 ------------------------------- BigQuery (MQTT) Connection -------------------------------------

Overview
Conectarse a la computadora de Google via MQTT para que la computadora procese los datos y los mande a BigQuery

Pseudocódigo
0. Se verifica que no se esté en NO_WIFI_MODE, si no se salta todo
1. Asignar el servidor, el keepalive,  el callback 
2. Mientras el status esté en no conectado va a repetir los siguientes pasos:
3. Mandar un request de conexión al servidor MQTT con un identificador por cliente que se compone así: identificador + clave_disp



 ------------------------------- Poll Submits monitoring and charge control ----------------------------------

Overview
Lee del servidor de firebase si alguien ya contestó la encuesta para así activar los cargadores. De igual manera checa si todos los 
usuarios ya desconectaron su celular de la banca para desactivar los cargadores de nuevo y avsarle a firebase que se apagaron, también 
controla el valor de lo que se va a publicar en el campo de usuario_cargando en big query

Pseudocódigo
0. Se verifica que no se esté en NO_WIFI_MODE, si sí lo está entonces se dejan los cargadores encendidos, de lo contrario sigue la lógica de abajo:
1. En caso de que haya un usuario ya cargándose:
	1.1 Se verifica que todavía está conectado, usando los sensores de corriente y checando si USER_NOT_CHARGING_TIMEOUT ya expiró
	1.2 En caso de que ya se haya desconectado el usuario, se escribe en firebase y se deshabilitan los cargadores
		(no se escribe en usuario_cargando en big query porque el objetivo es saber si alguien cargó al menos un ratito antes
		de que se vuelva a publicar en big query. En caso de que se pusiera en false se perdería ese true)
2. En caso de que no haya ningún usuario ya cargándose:
	2.1 Se lee si alguien ya contestó la encuesta en firebase
	2.2 En caso de que alguien la haya contestado, se escribe en usuario_cargando en big query y se encienden los cargadores


 ----------------------- Voltage and sensor processing ---------------------------------------

Overview 
Se leen los voltajes del panel y de la batería que van a ser publicados en big query, también se promedian para evitar publicar valores
basura 

Pseudocódigo
1. Se lee el valor de voltaje del panel o de la batería
2. Se hace un filtrado de ese valor para reconocer si es un valor basura o no
3. Se sobreescribe el promedio de valores que se llevan sumando desde la última vez que se publicaron los valores de los sensores

 ------------------------- WI FI Troubleshooting ----------------------------------------

Overview
Cada vez que se vaya a publicar algo en bigquery se hace un chequeo de la conexión a internet en el que 
se repiten los pasos del pseudocódigo de Wi Fi connection

 - NO_WIFI_MODE
En caso de que el dispositivo lleve cierto tiempo intentándose conectar y no lo haya logrado se pone en este modo
Este modo activa todos los cargadores y guarda los datos no mandados de manera interna hasta que se recupere la conexión. 
El sistema va a intentar salir de este modo pero sin afectar la ejecución del resto del programa

Se deshabilitan funciones como:
- Poll sumbits monitoring and charge control
- Sensor data publishing 

-------------------------- Sensor data publishing -------------------------------------------------

Overview
Preparación de los datos para ser mandados al servidor MQTT y que el servidor los publique en big query

Pseudocódigo
0. Se verifica que no esté en NO_WIFI_MODE, si sí lo está se salta toda la lógica 
1. Si ya pasó el tiempo definido en BIGQUERY_PUBLISHING empieza a ejecutar la siguiente lógica:
2. Se reinicia el countdown de BIGQUERY_PUBLISHING
3. Se les da el formato adecuado a las variables de voltaje para que puedan ser mandadas
4. Se hace el cálculo de la hora 
5. Se hace el cálculo de la fecha
6. Se hace el cálculo del folio
7. Se hace la lectura de usuario_cargando
8. Se publican todos los datos en bigquery
9. Se pone en "false" la variable de usuario_cargando

