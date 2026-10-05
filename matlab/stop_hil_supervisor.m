function stop_hil_supervisor()
% stop_hil_supervisor — cleanly stops the HIL supervisor poll timer and
% closes its TCP server socket so the port is freed and re-running
% hil_run_supervisor works without an "address already in use" error.

    t = timerfindall('Name', 'hil_supervisor_timer');
    if ~isempty(t)
        stop(t);
        delete(t);
        fprintf('[supervisor] poll timer stopped.\n');
    end

    if evalin('base', "exist('hil_supervisor_sock','var')")
        sock = evalin('base', 'hil_supervisor_sock');
        try, sock.close(); catch, end
        evalin('base', "clear hil_supervisor_sock");
        fprintf('[supervisor] socket closed, port freed.\n');
    end

    if evalin('base', "exist('hil_supervisor_timer_obj','var')")
        evalin('base', "clear hil_supervisor_timer_obj");
    end
end
