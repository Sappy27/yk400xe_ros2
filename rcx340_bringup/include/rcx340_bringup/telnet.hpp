#ifndef TELNET_HPP
#define TELNET_HPP

#define ASIO_STANDALONE

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <array>
#include <deque>
#include <functional>

#include <asio.hpp>
#include <asio/ts/buffer.hpp>

class TelnetCommunication{
    public:
    
        TelnetCommunication(const std::string& ip, const unsigned int& port, std::function<void(const std::string&)> cb = nullptr)
            : m_ip(ip)
            , m_port(port)
            , m_context()
            , m_work(asio::make_work_guard(m_context))
            , m_thread()
            , m_strand(asio::make_strand(m_context))
            , m_ec()
            , m_socket(m_context)
            , m_lineCallback(std::move(cb)){            
        }

        // Start the connection to controller
        void start(){
            // Start the context thread and run it
            m_thread = std::thread([this]() {m_context.run();});

            // Post connection to ASIO thread
            asio::post(m_strand, [this]() {
                connect();
            });
        }

        // Stop the connection 
        void stop(){
            disconnect();
        }

        
        // Send an UNIQUE command to the controller
        void send_command(std::string command)  
        {
            std::string msg;
            msg.reserve(command.size() + 2);
            msg.append(command);
            msg.append("\r\n", 2);

            command = std::move(msg);

            asio::post(m_strand, [this, cmd = std::move(command)]() mutable {
                bool idle = m_pendingCommands.empty();
                m_pendingCommands.push_back(std::move(cmd));

                if (idle) {
                    write_data();
                }
            });
        }

        // Send a BATCH of commands to the controller
        void send_command(std::vector<std::string> batch)   
        {
            for (auto& command : batch) {
                std::string msg;
                msg.reserve(command.size() + 2);
                msg.append(command);
                msg.append("\r\n", 2);

                command = std::move(msg);
            }

            asio::post(m_strand, [this, batch = std::move(batch)]() mutable {
                bool idle = m_pendingCommands.empty();

                for (auto& cmd : batch) {
                    m_pendingCommands.push_back(std::move(cmd));
                }

                if (idle) {
                    write_data();
                }
            });
        }

    private:
        std::string m_ip; // Controller IP
        unsigned int m_port; // Controller port

        asio::io_context m_context; // ASIO context
        asio::executor_work_guard<asio::io_context::executor_type> m_work; // Context work guard  
        std::thread m_thread; // Context thread

        asio::strand<asio::io_context::executor_type> m_strand; // strand for thread safe

        asio::error_code m_ec;  // Error code

        asio::ip::tcp::socket m_socket; // Controller socket


        std::array<char, 4096> m_readBuffer; // Buffer for reading
        std::string m_rxBuffer; // Buffer for lines

        std::deque<std::string> m_pendingCommands; // deque to store the pending commands

        std::function<void(const std::string&)> m_lineCallback;

        // Connect to the controller
        void connect(){

            // Connect the socket to the endpoint
            asio::ip::tcp::endpoint m_endpoint(asio::ip::make_address(m_ip,m_ec), m_port);
            if (m_ec) {
                std::cerr << "Invalid IP: " << m_ec.message() << std::endl;
                return;
            }

            m_socket.connect(m_endpoint,m_ec);

            if (!m_ec){
                std::cout << "Connected to the controller at : " << m_ip << ":" << m_port << std::endl;
                read_data();
            }
            else{
                std::cerr << "Failed to connect to the controller : " << m_ec.message() << std::endl;
            }

            
        }

        void disconnect(){
            // Close the socket in the context thread
            asio::post(m_strand, [this]() {
                asio::error_code ec;
                m_socket.close(ec);
            });

            // Stop the work guard so the context can stop 
            m_work.reset();
            // Stop the context
            m_context.stop();

            // Wait for the thread to stop
            if (m_thread.joinable()) m_thread.join();
        }

        // Async // Read data coming from the controller
        void read_data(){
            // Read data asynchronously on the context thread
            m_socket.async_read_some(asio::buffer(m_readBuffer.data(),m_readBuffer.size()),
                asio::bind_executor(m_strand,[this] (const asio::error_code& ec, size_t lenght ){

                    if(ec){
                        std::cerr << "Error while reading : " << ec.message() << std::endl;
                        return;
                    }

                    m_rxBuffer.append(m_readBuffer.data(),lenght);

                    // Extract complete lines
                    std::size_t pos;
                    while ((pos = m_rxBuffer.find('\n')) != std::string::npos) {
                        std::string line = m_rxBuffer.substr(0, pos);

                        // Remove CR if present (\r\n)
                        if (!line.empty() && line.back() == '\r') {
                            line.pop_back();
                        }

                        m_rxBuffer.erase(0, pos + 1);

                        // Process one full line
                        on_line_received(line);
                    }

                    // Continue reading
                    read_data();
                })
            );
        }

        // Proccess received lines 
        void on_line_received(std::string line)
        {
            if (m_lineCallback) {
                m_lineCallback(line);
            }
        }

        // Async // Write data to the controller
        void write_data(){
            // If no command to write return
            if (m_pendingCommands.empty()){
                return;
            }

            // Grab the first command in the deque
            const std::string& msg = m_pendingCommands.front();

            // Write the message asynchronously on the context thread
            m_socket.async_write_some(asio::buffer(msg.data(),msg.size()),
               asio::bind_executor(m_strand, [this] (asio::error_code ec, size_t /*lenght*/){

                    if (ec){
                        std::cerr << "Error while writing : " << ec.message() << std::endl;
                    }

                    // Pop the command from the deque 
                    m_pendingCommands.pop_front();

                    if (!m_pendingCommands.empty()){
                        // Continue writing
                        write_data();
                    }
                })
            );
        }


};

#endif // TELNET_HPP