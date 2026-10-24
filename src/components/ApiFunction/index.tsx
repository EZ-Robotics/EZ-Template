// Renders one function from the header comments, in the same shape as a hand-written entry:
// summary, details, param lines, then Prototype and Example tabs.
//
//   ### pid_wait()
//
//   <ApiFunction name="pid_wait">
//
//   ```cpp
//   chassis.pid_wait();
//   ```
//
//   </ApiFunction>
//
// The page keeps its own "### heading" so the table of contents still lists the function. The
// example inside the tag is used when the header has no \code example. How name and args are
// matched is in scripts/api/resolve.mjs. The text comes from the snapshot of the docs version the page
// belongs to (api/config.json), which `npm run api:update` makes.

import React, {type ReactNode} from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';
import Admonition from '@theme/Admonition';
import {useDocsVersion} from '@docusaurus/plugin-content-docs/client';
import apiData from '@site/api/generated/data.json';
import {resolveFunction} from '../../../scripts/api/resolve.mjs';

type Inline = string | {t: 'code'; c: string} | {t: 'b' | 'i'; c: Inline[]} | {t: 'a'; href: string; c: Inline[]} | {t: 'br'};
type Block =
  | {t: 'p'; c: Inline[]}
  | {t: 'ul' | 'ol'; items: Block[][]}
  | {t: 'pre'; code: string}
  | {t: 'admonition'; type: 'note' | 'warning' | 'info' | 'danger'; title: string | null; c: Block[]};

interface ApiFn {
  anchor: string;
  prototype: string;
  brief: Inline[] | null;
  details: Block[];
  paramDocs: {name: string; desc: Block[]}[];
  returns: Block[];
  examples: {lang: string; code: string}[];
}

interface Props {
  readonly name: string;
  readonly args?: string;
  readonly children?: ReactNode;
}

function Inlines({nodes}: {nodes: Inline[]}): ReactNode {
  return (
    <>
      {nodes.map((n, i) => {
        if (typeof n === 'string') return n;
        switch (n.t) {
          case 'code':
            return <code key={i}>{n.c}</code>;
          case 'b':
            return <strong key={i}><Inlines nodes={n.c} /></strong>;
          case 'i':
            return <em key={i}><Inlines nodes={n.c} /></em>;
          case 'a':
            return <a key={i} href={n.href}><Inlines nodes={n.c} /></a>;
          case 'br':
            return <br key={i} />;
        }
      })}
    </>
  );
}

// The running text of a block list, for places that sit inside one line (a param, "Returns").
function flatText(blocks: Block[]): Inline[] {
  return blocks.flatMap((b, i) => (b.t === 'p' ? [...(i ? [' '] : []), ...b.c] : []));
}

function Blocks({blocks}: {blocks: Block[]}): ReactNode {
  return (
    <>
      {blocks.map((b, i) => {
        switch (b.t) {
          case 'p':
            return <p key={i}><Inlines nodes={b.c} /></p>;
          case 'ul':
          case 'ol': {
            const List = b.t;
            return (
              <List key={i}>
                {b.items.map((item, j) => (
                  <li key={j}>{item.length === 1 && item[0].t === 'p' ? <Inlines nodes={item[0].c} /> : <Blocks blocks={item} />}</li>
                ))}
              </List>
            );
          }
          case 'pre':
            return <CodeBlock key={i}>{b.code}</CodeBlock>;
          case 'admonition':
            return (
              <Admonition key={i} type={b.type} title={b.title ?? undefined}>
                <Blocks blocks={b.c} />
              </Admonition>
            );
        }
      })}
    </>
  );
}

export default function ApiFunction({name, args, children}: Props): ReactNode {
  // Each docs version reads its own snapshot; prepare.mjs wrote just the functions its pages place.
  const version = useDocsVersion().version;
  const functions = (apiData as Record<string, ApiFn[]>)[version];
  if (!functions) {
    throw new Error(`<ApiFunction name="${name}"> has no API data for docs version "${version}". Map that version to a snapshot in api/config.json.`);
  }
  const fn = resolveFunction(functions, {name, args}) as ApiFn;
  const hasExample = fn.examples.length > 0 || React.Children.count(children) > 0;
  const prototype = <CodeBlock language="cpp">{fn.prototype}</CodeBlock>;

  return (
    <div id={fn.anchor}>
      {fn.brief && (
        <p>
          <Inlines nodes={fn.brief} />
        </p>
      )}
      <Blocks blocks={fn.details} />
      {fn.paramDocs.length > 0 && (
        <p>
          {fn.paramDocs.map((p, i) => (
            <React.Fragment key={p.name}>
              {i > 0 && <br />}
              <code>{p.name}</code> <Inlines nodes={flatText(p.desc)} />
            </React.Fragment>
          ))}
        </p>
      )}
      {fn.returns.length > 0 && (
        <p>
          <strong>Returns</strong> <Inlines nodes={flatText(fn.returns)} />
        </p>
      )}
      {hasExample ? (
        <Tabs
          groupId={fn.anchor}
          defaultValue="proto"
          values={[
            {label: 'Prototype', value: 'proto'},
            {label: 'Example', value: 'example'},
          ]}>
          <TabItem value="proto">{prototype}</TabItem>
          <TabItem value="example">
            {fn.examples.length > 0
              ? fn.examples.map((e, i) => (
                  <CodeBlock key={i} language={e.lang}>
                    {e.code}
                  </CodeBlock>
                ))
              : children}
          </TabItem>
        </Tabs>
      ) : (
        prototype
      )}
    </div>
  );
}
