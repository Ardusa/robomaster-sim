/**
 * Persistent MJPEG camera panes via web_video_server /stream.
 *
 * Snapshot polling (/snapshot every frame) creates and tears down a streamer
 * per request — that floods the bringup log and flickers the UI. One long-lived
 * <img> per topic keeps a single stream open.
 */

export function createStreamFeed(container) {
  container.classList.add("feed", "feed-stack");
  container.replaceChildren();

  const img = document.createElement("img");
  img.className = "feed-layer active";
  img.alt = container.dataset.alt || "camera";
  img.decoding = "async";
  container.append(img);

  let stopped = false;

  function start(videoBase, topicName, title) {
    if (stopped) return;
    const base = String(videoBase || "").replace(/\/$/, "");
    const topic = topicName || "";
    if (title) {
      img.alt = title;
      container.dataset.alt = title;
    }
    if (!base || !topic) {
      img.removeAttribute("src");
      return;
    }
    // web_video_server wants a literal ROS topic (/camera/...), not %2F-encoded.
    // Topics come from our config, not free-form user input.
    img.src = `${base}/stream?topic=${topic}&type=mjpeg&quality=60`;
  }

  function dispose() {
    stopped = true;
    img.removeAttribute("src");
  }

  return { start, dispose };
}

/** @deprecated Use createStreamFeed — kept as an alias for older imports. */
export const createSnapshotFeed = createStreamFeed;
