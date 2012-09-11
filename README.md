From github.com/jcw/jeelib

Added function to set RFM12B CS pin in a sketch. Usfull when using alternative hardware setups like the Open Kontrol Gateway

When using this library the line

rf12_set_cs(9) //9 for Kontrol Gateway - change to (10) for JeeNode/emonTx and NanodeRF

must be added in setup before rf12initialize. 
