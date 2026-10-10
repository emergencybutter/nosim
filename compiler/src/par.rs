//! Order-preserving parallel map over scoped threads (standard library only).

use crate::CompileError;

/// Worker count: `requested`, or every available core when it is 0.
pub fn thread_count(requested: usize) -> usize {
    if requested == 0 { std::thread::available_parallelism().map_or(1, usize::from) } else { requested }
}

/// Applies `f` to every item on `threads` scoped workers that take the next unclaimed item,
/// returning results in input order. Stops handing out work after the first error and
/// returns the error from the earliest failing item.
pub fn parallel_map<T: Sync, R: Send>(
    items: &[T],
    threads: usize,
    f: impl Fn(&T) -> Result<R, CompileError> + Sync,
) -> Result<Vec<R>, CompileError> {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    let next = AtomicUsize::new(0);
    let failed = AtomicBool::new(false);
    let slots: Vec<Mutex<Option<Result<R, CompileError>>>> = items.iter().map(|_| Mutex::new(None)).collect();
    std::thread::scope(|scope| {
        for _ in 0..threads.clamp(1, items.len().max(1)) {
            scope.spawn(|| {
                while !failed.load(Ordering::Relaxed) {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(item) = items.get(i) else { break };
                    let r = f(item);
                    if r.is_err() {
                        failed.store(true, Ordering::Relaxed);
                    }
                    *slots[i].lock().expect("no panics while holding the slot") = Some(r);
                }
            });
        }
    });
    let mut out = Vec::with_capacity(items.len());
    for slot in slots {
        match slot.into_inner().expect("workers joined") {
            Some(Ok(r)) => out.push(r),
            Some(Err(e)) => return Err(e),
            None => {} // never claimed because an earlier item failed; that error follows
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parallel_map_keeps_order_and_reports_errors() {
        let items: Vec<u32> = (0..1000).collect();
        for threads in [1, 3, 8] {
            let out = parallel_map(&items, threads, |&i| Ok(i * 2)).unwrap();
            assert_eq!(out, items.iter().map(|i| i * 2).collect::<Vec<_>>());
        }
        let err = parallel_map(&items, 4, |&i| if i == 500 { Err(CompileError::Raster("boom".into())) } else { Ok(i) });
        assert!(matches!(err, Err(CompileError::Raster(m)) if m == "boom"));
        assert_eq!(parallel_map(&[] as &[u32], 4, |&i| Ok(i)).unwrap(), Vec::<u32>::new());
    }
}
