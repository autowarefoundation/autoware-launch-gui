import { useCallback, useEffect, useRef, useState } from "react";

function useDebouncedState<T>(
  initialValue: T,
  delay: number
): [T, (value: T) => void] {
  const [state, setState] = useState(initialValue);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  const setDebouncedState = useCallback(
    (value: T) => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }

      timeoutRef.current = setTimeout(() => {
        setState(value);
        timeoutRef.current = null;
      }, delay);
    },
    [delay]
  );

  useEffect(() => {
    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, []);

  return [state, setDebouncedState];
}

export default useDebouncedState;
