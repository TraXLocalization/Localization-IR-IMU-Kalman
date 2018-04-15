function [s] = server(host, port)
        
    s.Q = Queue();
    s.socket = tcpip(host,port,'NetworkRole','Server');
    set(s.socket, 'InputBufferSize', 3000000);
    set(s.socket, 'OutputBufferSize', 3000000);
    fopen(s.socket);

    s.send = @send;
    function send(data)
        d = whos(data);
        fwrite(s.socket, data, d.class);
    end

    s.receive = @receive;
    function receive()
        TimerFcn = {@recv, s.socket, s.Q};
        t = timer('ExecutionMode', 'FixedRate', ...
            'Period', 1, ...
            'TimerFcn', TimerFcn);
        start(t);
    end

    s.disconnect = @disconnect;
    function disconnect()
        fclose(s.socket);
        delete(s.socket);
        clear s.socket;
    end
    
    s.message = @message;
    function y = message()
        y = str2double(strsplit(fgetl(s.socket),','));
    end
end

function Q = recv(hobj, eventdata, socket, Q)
    while(socket.BytesAvailable > 0)
        data_str = fgetl(socket);
        data_num = str2double(strsplit(data_str,',')); %parse data creating an array
        Q.enqueue(data_str);
        disp("Server: " + data_num);
        %data_num(1) = randi(10); data_num(2) = randi(10);
        plotposition(data_num);
    end
    
end