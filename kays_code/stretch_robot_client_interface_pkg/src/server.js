/*

const https = require('https');

const fs = require('fs');

const cert = fs.readFileSync('certificate.pem');
const ca = fs.readFileSync('8ECDE6884F3D87B1125BA31AC3FCB13D7016DE7F57CC904FE1CB97C6AE98196E.crt');
const key = fs.readFileSync('private-key.pem');

const hostname = '127.0.0.1';
const port = 8443;

const rosnodejs = require('rosnodejs');
rosnodejs.initNode('/my_node')
.then(() => {
	const nh = rosnodejs.nh;

	const sub = nh.subscribe('/client_command', 'std_msgs/String', (msg) => {
	  console.log('Got msg from alexa: %j', msg);
	});
});

let options = {
	cert: cert, // fs.readFileSync('./ssl/example.crt');
	ca: ca,
	key: key // fs.readFileSync('./ssl/example.key');
 };

const server = https.createServer(options, (req, res) => {
  res.statusCode = 200;
  res.setHeader('Content-Type', 'text/plain');
  res.end('Hello World');
});

server.listen(port, hostname, () => {
  console.log(`Server running at https://${hostname}:${port}/`);
});

*/