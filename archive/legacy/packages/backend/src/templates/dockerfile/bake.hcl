group "default" { 
  targets = [{{#each targets}}"{{this}}"{{#unless @last}}, {{/unless}}{{/each}}] 
}

{{#each stages}}
target "{{id}}" {
  dockerfile = "{{dockerfile}}"
  {{#if args}}
  args = { {{#each args}}{{@key}} = "{{this}}"{{#unless @last}}, {{/unless}}{{/each}} }
  {{/if}}
  tags = [{{#each tags}}"{{this}}"{{#unless @last}}, {{/unless}}{{/each}}]
  {{#if context}}
  context = "{{context}}"
  {{/if}}
  {{#if platforms}}
  platforms = [{{#each platforms}}"{{this}}"{{#unless @last}}, {{/unless}}{{/each}}]
  {{/if}}
}
{{/each}}

