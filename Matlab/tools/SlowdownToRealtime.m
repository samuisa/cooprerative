function SlowdownToRealtime(dt)
%SLOWDOWNTOREALTIME Busy-wait until one real-time sample period has elapsed.
%
% Input:
%   dt - Desired wall-clock interval between consecutive calls [s].
%
% Output:
%   None. The function blocks until toc >= dt, then restarts the shared tic/toc stopwatch for the next iteration.
%
% A tic call must initialize the stopwatch before the first invocation. This helper intentionally uses a busy wait and is meant only for simple teaching simulations; it consumes CPU while waiting.

while toc < dt
end

tic
end
