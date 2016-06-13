#![allow(dead_code)]

use std::sync::mpsc::{channel, Sender, Receiver};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::{Builder, panicking};

trait FnBox {
    fn call_box(self: Box<Self>);
}

impl<F: FnOnce()> FnBox for F {
    fn call_box(self: Box<F>) {
        (*self)()
    }
}


type Thunk<'a> = Box<FnBox + Send + 'a>;

struct Sentinel<'a> {
    name: Option<String>,
    stack_size: Option<usize>,
    jobs: &'a Arc<Mutex<Receiver<Thunk<'static>>>>,
    thread_counter: &'a Arc<AtomicUsize>,
    thread_count_max: &'a Arc<AtomicUsize>,
    thread_count_panic: &'a Arc<AtomicUsize>,
    active: bool,
}

impl<'a> Sentinel<'a> {
    fn new(name: Option<String>,
           stack_size: Option<usize>,
           jobs: &'a Arc<Mutex<Receiver<Thunk<'static>>>>,
           thread_counter: &'a Arc<AtomicUsize>,
           thread_count_max: &'a Arc<AtomicUsize>,
           thread_count_panic: &'a Arc<AtomicUsize>)
           -> Sentinel<'a> {
        Sentinel {
            name: name,
            stack_size: stack_size,
            jobs: jobs,
            thread_counter: thread_counter,
            thread_count_max: thread_count_max,
            thread_count_panic: thread_count_panic,
            active: true,
        }
    }

    fn cancel(mut self) {
        self.active = false;
    }
}

impl<'a> Drop for Sentinel<'a> {
    fn drop(&mut self) {
        if self.active {
            self.thread_counter.fetch_sub(1, Ordering::SeqCst);
            if panicking() {
                self.thread_count_panic.fetch_add(1, Ordering::SeqCst);
            }
            spawn_in_pool(self.name.clone(),
                          self.stack_size.clone(),
                          self.jobs.clone(),
                          self.thread_counter.clone(),
                          self.thread_count_max.clone(),
                          self.thread_count_panic.clone())
        }
    }
}


macro_rules! builder {
    ( $src_name: ident => $dest_name: ident {
        $( $attr_name: ident : $attr_type: ty = $attr_default: expr ),*
    })
    => {
        pub struct $src_name {
            $( $attr_name: Option<$attr_type> ),*
        }

        impl $src_name {
            pub fn new() -> $src_name {
                $src_name {
                    $(
                        $attr_name: $attr_default
                    ),*
                }
            }

            $(
                pub fn $attr_name(mut self, value: $attr_type) -> Self {
                    self.$attr_name = Some(value);
                    self
                }
            )*
        }
    }
}

builder!(
    ThreadPoolBuilder => ThreadPool {
        name: String = None,
        stack_size: usize = None,
        num_threads: usize = Some(2)
    }
);

impl ThreadPoolBuilder {
    #[inline]
    pub fn build(self) -> ThreadPool {
        ThreadPool::builderinterface(self.name, self.stack_size, self.num_threads.unwrap())
    }
}

#[derive(Clone)]
pub struct ThreadPool {
    name: Option<String>,
    stack_size: Option<usize>,
    jobs: Sender<Thunk<'static>>,
    job_reciver: Arc<Mutex<Receiver<Thunk<'static>>>>,
    active_count: Arc<AtomicUsize>,
    max_count: Arc<AtomicUsize>,
    panic_count: Arc<AtomicUsize>,
}

impl ThreadPool {
    pub fn new(num_threads: usize) -> ThreadPool {
        Self::new_pool(None, None, num_threads)
    }
    pub fn new_with_name(name: String, num_threads: usize) -> ThreadPool {
        Self::new_pool(Some(name), None, num_threads)
    }

    fn builderinterface(name: Option<String>,
                        stack_size: Option<usize>,
                        num_threads: usize)
                        -> ThreadPool {
        Self::new_pool(name, stack_size, num_threads)
    }
    pub fn execute<F>(&self, job: F)
        where F: FnOnce() + Send + 'static
    {
        self.jobs.send(Box::new(move || job())).unwrap();
    }
    pub fn active_count(&self) -> usize {
        self.active_count.load(Ordering::Relaxed)
    }
    pub fn max_count(&self) -> usize {
        self.max_count.load(Ordering::Relaxed)
    }
    pub fn panic_count(&self) -> usize {
        self.panic_count.load(Ordering::Relaxed)
    }
    pub fn set_threads(&mut self, num_threads: usize) {
        self.set_num_threads(num_threads)
    }
    pub fn set_num_threads(&mut self, num_threads: usize) {
        assert!(num_threads >= 1);
        let prev_num_threads = (*self.max_count).swap(num_threads, Ordering::Release);
        if let Some(num_spawn) = num_threads.checked_sub(prev_num_threads) {
            for _ in 0..num_spawn {
                spawn_in_pool(self.name.clone(),
                              self.stack_size.clone(),
                              self.job_reciver.clone(),
                              self.active_count.clone(),
                              self.max_count.clone(),
                              self.panic_count.clone());
            }
        }
    }
    #[inline]
    fn new_pool(name: Option<String>, stack_size: Option<usize>, num_threads: usize) -> ThreadPool {
        assert!(num_threads >= 1);
        let (tx, rx) = channel::<Thunk<'static>>();
        let rx = Arc::new(Mutex::new(rx));
        let active_count = Arc::new(AtomicUsize::new(0));
        let max_count = Arc::new(AtomicUsize::new(num_threads));
        let panic_count = Arc::new(AtomicUsize::new(0));

        for _ in 0..num_threads {
            spawn_in_pool(name.clone(),
                          stack_size.clone(),
                          rx.clone(),
                          active_count.clone(),
                          max_count.clone(),
                          panic_count.clone());
        }

        ThreadPool {
            name: name,
            stack_size: stack_size,
            jobs: tx,
            job_reciver: rx.clone(),
            active_count: active_count,
            max_count: max_count,
            panic_count: panic_count,
        }
    }
}

fn spawn_in_pool(name: Option<String>,
                 stack_size: Option<usize>,
                 jobs: Arc<Mutex<Receiver<Thunk<'static>>>>,
                 thread_counter: Arc<AtomicUsize>,
                 thread_count_max: Arc<AtomicUsize>,
                 thread_count_panic: Arc<AtomicUsize>) {
    let mut builder = Builder::new();
    if let Some(ref name) = name {
        match stack_size {
            Some(ref stack_size) => builder = builder.name(name.clone()).stack_size(*stack_size),
            None => builder = builder.name(name.clone()),
        }
    }
    builder.spawn(move || {
            let sentinel = Sentinel::new(name,
                                         stack_size,
                                         &jobs,
                                         &thread_counter,
                                         &thread_count_max,
                                         &thread_count_panic);
            loop {
                let thread_count_val = thread_counter.load(Ordering::Acquire);
                let thread_count_max_val = thread_count_max.load(Ordering::Relaxed);
                if thread_count_val < thread_count_max_val {
                    let message = {
                        let lock = jobs.lock().unwrap();
                        lock.recv()
                    };

                    match message {
                        Ok(job) => {
                            thread_counter.fetch_add(1, Ordering::SeqCst);
                            job.call_box();
                            thread_counter.fetch_sub(1, Ordering::SeqCst);
                        }
                        Err(..) => break,
                    }
                } else {
                    break;
                }
            }
            sentinel.cancel();
        })
        .unwrap();
}
