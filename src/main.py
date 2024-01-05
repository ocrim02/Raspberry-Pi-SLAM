from multiprocessing import Queue, Process
from processes.gather_process import gather_process
from processes.loop_closure_process import loop_closure_process
from processes.mapping_process import mapping_process

if __name__ == '__main__':
    icp_queue = Queue()
    map_queue = Queue()
    og_queue = Queue()

    # init new process for gathering and loop closure
    gather = Process(target=gather_process, args=(icp_queue,))
    loop_closure = Process(target=loop_closure_process, args=(icp_queue, map_queue, og_queue,))
    gather.start()
    loop_closure.start()

    try:
        mapping_process(map_queue, og_queue)
    except KeyboardInterrupt:
        print("kill child processes")
        gather.terminate()
        loop_closure.terminate()
        #display.terminate()
    gather.terminate()
    loop_closure.terminate()
    #p.join()