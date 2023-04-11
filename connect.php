<html>
<body>

<?php

$dbname = 'id20432339_ustp_apm';
$dbuser = 'id20432339_ustpapm';  
$dbpass = 'IJDOq_)9d2m]74-)'; 
$dbhost = 'localhost'; 

$connect = @mysqli_connect($dbhost,$dbuser,$dbpass,$dbname);

if(!$connect){
	echo "Error: " . mysqli_connect_error();
	exit();
}

echo "Connection Success!<br><br>";

$PM1 = $_GET["PM1"];
$PM2_5 = $_GET["PM2_5"];
$PM10 = $_GET["PM10"];
$temperature = $_GET["temperature"];
$humidity = $_GET["humidity"];
$CO = $_GET["CO"]; 
$PM25_aqival = $_GET["PM25_aqival"]; 
$aqi_status = $_GET["aqi_status"]; 


$query = "INSERT INTO sensor_data (PM1, PM2_5, PM10, temperature, humidity, CO, PM25_aqival, aqi_status) VALUES ('$PM1', '$PM2_5', '$PM10', '$temperature', '$humidity', '$CO', '$PM25_aqival', '$aqi_status')";
$result = mysqli_query($connect,$query);

echo "Insertion Success!<br>";

?>
</body>
</html>