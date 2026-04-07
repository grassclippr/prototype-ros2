package main

import (
	"encoding/hex"
	"fmt"
	"io"
	"net"
)

func main() {
	// listen to port tcp 8888 and print all received data with hex dump

	l, err := net.Listen("tcp", ":8888")
	if err != nil {
		fmt.Println("Error listening:", err.Error())
		return
	}

	defer l.Close()

	fmt.Println("Listening on :8888")
	for {
		conn, err := l.Accept()
		if err != nil {
			fmt.Println("Error accepting: ", err.Error())
			return
		}
		go handleRequest(conn)
	}

	select {}
}

func handleRequest(conn net.Conn) {
	defer conn.Close()
	buf := make([]byte, 1024)
	for {
		n, err := conn.Read(buf)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Error reading:", err.Error())
			}
			break
		}

		fmt.Printf("Received %d bytes (exp %d):\n%s\n", n, buf[0]<<8+buf[1], hex.Dump(buf[:n]))
	}
}
