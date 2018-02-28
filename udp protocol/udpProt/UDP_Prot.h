#ifndef UDP_PROT_HPP
#define UDP_PROT_HPP

#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h> // socket


class UDP_Server
{
public:
	UDP_Server();
	~UDP_Server();

	bool IsInit() { return m_socket != INVALID_SOCKET; };
	bool Init(int portNum);

	int Read(char * buffer);
	bool Write(char * buffer, int size);
	int TimeOut(int ms);

	static const int s_BUF_LEN = 2048;

private:
	SOCKET m_socket;
	struct sockaddr_in m_siOther;
};


class UDP_Client
{
public:
public:
	UDP_Client();
	~UDP_Client();

	bool IsInit() { return m_socket != INVALID_SOCKET; };
	bool Init(int portNum, char *host = "127.0.0.1");

	int Read(char * buffer);
	bool Write(char * buffer, int size);
	int TimeOut(int ms);

private:
	SOCKET m_socket;
	struct sockaddr_in m_server;
};

# endif // TCP_PROT_HPP