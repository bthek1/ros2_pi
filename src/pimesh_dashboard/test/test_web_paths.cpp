// The half of an HTTP server that is wrong without anything crashing.
//
// **This page is bound to the LAN on purpose** — `docs/info/dashboard.md` says a
// phone on the same network should be able to watch — so "nobody can reach it" is
// not a defence and never was. `GET /../../../../etc/passwd` is a request no
// browser sends and `curl --path-as-is` does, and a server that serves it does so
// with a 200 and no log line saying anything was unusual.
//
// The content type half is quieter still: a file served as the wrong type either
// does not render (a stylesheet as octet-stream is ignored by every browser, with
// nothing in the console) or renders when it should not.

#include <gtest/gtest.h>

#include <string>

#include "pimesh_dashboard/web_server.hpp"

using pimesh_dashboard::content_type_for;
using pimesh_dashboard::resolve_path;

namespace
{
const char * kRoot = "/opt/pimesh/share/web";
}

TEST(ResolvePath, ServesTheIndexForTheRoot)
{
  EXPECT_EQ(resolve_path(kRoot, "/"), std::string(kRoot) + "/index.html");
  EXPECT_EQ(resolve_path(kRoot, "/app.js"), std::string(kRoot) + "/app.js");
  EXPECT_EQ(resolve_path(kRoot, "/sub/thing.css"), std::string(kRoot) + "/sub/thing.css");
}

TEST(ResolvePath, StripsTheQueryAndTheFragment)
{
  // A cache-buster is the ordinary reason a real page asks for `app.js?v=3`, and
  // a server that takes the whole thing as a filename answers 404 for a file that
  // is right there.
  EXPECT_EQ(resolve_path(kRoot, "/app.js?v=3"), std::string(kRoot) + "/app.js");
  EXPECT_EQ(resolve_path(kRoot, "/index.html#top"), std::string(kRoot) + "/index.html");
  EXPECT_EQ(resolve_path(kRoot, "/?x=1"), std::string(kRoot) + "/index.html");
}

TEST(ResolvePath, RefusesAnythingThatLeavesTheRoot)
{
  // **Refused, not clamped.** Popping the `..` would serve a *different* file
  // than the one asked for, which makes the access log and the behaviour
  // disagree — and nothing this page serves ever contains one.
  for (const char * target : {
      "/../etc/passwd",
      "/../../../../etc/passwd",
      "/sub/../../etc/passwd",
      "/..",
      "/a/b/../../../c",
    })
  {
    EXPECT_TRUE(resolve_path(kRoot, target).empty()) << target;
  }
}

TEST(ResolvePath, RefusesWhatIsNotARelativeFilePath)
{
  EXPECT_TRUE(resolve_path(kRoot, "").empty());
  EXPECT_TRUE(resolve_path(kRoot, "app.js").empty()) << "a target must start with /";
  EXPECT_TRUE(resolve_path(kRoot, "http://elsewhere/x").empty());
  // A NUL truncates every C string the path later passes through, so `/a\0/../b`
  // would be checked as one thing and opened as another.
  EXPECT_TRUE(resolve_path(kRoot, std::string("/ok\0/../../etc", 14)).empty());
  // No root configured is not "serve from /".
  EXPECT_TRUE(resolve_path("", "/index.html").empty());
}

TEST(ResolvePath, CollapsesHarmlessNoise)
{
  // `//` and `/./` are ordinary in a URL a page builds by concatenation, and are
  // not an attack. Refusing them would break the page; resolving them to
  // somewhere else would be the bug above.
  EXPECT_EQ(resolve_path(kRoot, "//app.js"), std::string(kRoot) + "/app.js");
  EXPECT_EQ(resolve_path(kRoot, "/./app.js"), std::string(kRoot) + "/app.js");
  EXPECT_EQ(resolve_path(kRoot, "/sub//./thing.css"), std::string(kRoot) + "/sub/thing.css");
}

TEST(ContentType, NamesTheTypesThisPageActuallyServes)
{
  EXPECT_EQ(content_type_for("/x/index.html"), "text/html; charset=utf-8");
  EXPECT_EQ(content_type_for("/x/style.css"), "text/css; charset=utf-8");
  EXPECT_EQ(content_type_for("/x/app.js"), "application/javascript; charset=utf-8");
  EXPECT_EQ(content_type_for("/x/data.json"), "application/json");
  EXPECT_EQ(content_type_for("/x/icon.png"), "image/png");
  EXPECT_EQ(content_type_for("/x/photo.JPG"), "image/jpeg") << "extensions are case-insensitive";
}

TEST(ContentType, UnknownExtensionsDownloadRatherThanRun)
{
  EXPECT_EQ(content_type_for("/x/thing"), "application/octet-stream");
  EXPECT_EQ(content_type_for("/x/thing.wat"), "application/octet-stream");
  EXPECT_EQ(content_type_for(""), "application/octet-stream");
}
