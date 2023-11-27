from kyonrlstepping.utils.rhc2ros import Shared2ROSInternal

from perf_sleep.pyperfsleep import PerfSleep

if __name__ == '__main__':

    robot_name = "kyon0"

    bridge = Shared2ROSInternal(namespace=robot_name, 
                        index=0, 
                        verbose=True, 
                        basename="RHC2SharedInternal")
    
    bridge.run()

    perf_timer = PerfSleep()

    while True:

        bridge.update()

        perf_timer.clock_sleep(1000000) # nanoseconds (actually resolution is much
                    # poorer)
