/**
 * Flicker-free camera panes.
 *
 * web_video_server's multipart MJPEG in a single <img> blanks between frames
 * under load (especially with the WebGL reconstruction). Snapshot polling into
 * a double-buffered pair keeps the last good frame on screen.
 */

export function createSnapshotFeed(container, { hz = 12 } = {}) {
  container.classList.add("feed", "feed-stack");
  container.replaceChildren();

  const a = document.createElement("img");
  const b = document.createElement("img");
  a.className = "feed-layer active";
  b.className = "feed-layer";
  a.alt = b.alt = container.dataset.alt || "camera";
  a.decoding = b.decoding = "async";
  container.append(a, b);

  let front = a;
  let back = b;
  let base = "";
  let topic = "";
  let timer = null;
  let busy = false;
  let blobUrl = null;
  let stopped = false;

  async function pull() {
    if (stopped || busy || document.hidden || !base || !topic) return;
    busy = true;
    try {
      const res = await fetch(
        `${base}/snapshot?topic=${topic}&_=${Date.now()}`,
        { cache: "no-store" }
      );
      if (!res.ok) return;
      const blob = await res.blob();
      if (!blob.size) return;
      const url = URL.createObjectURL(blob);
      await new Promise((resolve, reject) => {
        back.onload = () => resolve();
        back.onerror = () => reject(new Error("frame decode failed"));
        back.src = url;
      });
      back.classList.add("active");
      front.classList.remove("active");
      const prev = blobUrl;
      blobUrl = url;
      if (prev) URL.revokeObjectURL(prev);
      const tmp = front;
      front = back;
      back = tmp;
    } catch (_) {
      /* keep showing the last good frame */
    } finally {
      busy = false;
    }
  }

  function start(videoBase, topicName, title) {
    base = String(videoBase || "").replace(/\/$/, "");
    topic = topicName || "";
    if (title) {
      a.alt = b.alt = title;
      container.dataset.alt = title;
    }
    stopTimer();
    if (!base || !topic) return;
    pull();
    timer = setInterval(pull, Math.max(50, 1000 / hz));
  }

  function stopTimer() {
    if (timer != null) {
      clearInterval(timer);
      timer = null;
    }
  }

  function dispose() {
    stopped = true;
    stopTimer();
    if (blobUrl) URL.revokeObjectURL(blobUrl);
    blobUrl = null;
  }

  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) pull();
  });

  return { start, dispose };
}
