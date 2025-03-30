const express = require('express');
const bodyParser = require('body-parser');
const cors = require('cors');

const app = express();
const port = 3000;

app.use(cors());
app.use(bodyParser.json());

app.post('/test', (req, res) => {
    console.log('Received data:');
    console.log(req.body);
    res.status(200).json({ 
        message: 'Data received successfully',
        receivedData: req.body 
    });
});

app.listen(port, '0.0.0.0', () => {
    console.log(`Server running on port ${port}`);
});
